#include <asuka/modules/optimization_miao.hpp>

#include <chrono>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace optimization_miao {

OptimizationMIAO::OptimizationMIAO() {
  logger = create_module_logger("miao");

  const Config config(GlobalConfig::get_config_path("config_extensions"));
  verbose = config.param<bool>("optimization_miao", "verbose", true);
  loop_kf_gap = config.param<int>("optimization_miao", "loop_kf_gap", 20);
  min_id_interval = config.param<int>("optimization_miao", "min_id_interval", 20);
  closest_id_th = config.param<int>("optimization_miao", "closest_id_th", 50);
  max_range = config.param<double>("optimization_miao", "max_range", 30.0);
  ndt_score_th = config.param<double>("optimization_miao", "ndt_score_th", 1.0);
  motion_trans_noise = config.param<double>("optimization_miao", "motion_trans_noise", 0.1);
  motion_rot_noise = config.param<double>("optimization_miao", "motion_rot_noise", 3.0) * M_PI / 180.0;
  loop_trans_noise = config.param<double>("optimization_miao", "loop_trans_noise", 0.2);
  loop_rot_noise = config.param<double>("optimization_miao", "loop_rot_noise", 3.0) * M_PI / 180.0;
  rk_loop_th = config.param<double>("optimization_miao", "rk_loop_th", 5.2 / 5.0);
  with_height = config.param<bool>("optimization_miao", "with_height", true);
  height_noise = config.param<double>("optimization_miao", "height_noise", 0.1);
  publish_interval = 1.0 / config.param<double>("optimization_miao", "publish_rate", 10.0);

  ::lightning::miao::OptimizerConfig opt_config(
    ::lightning::miao::AlgorithmType::LEVENBERG_MARQUARDT,
    ::lightning::miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN,
    false);
  opt_config.incremental_mode_ = true;
  optimizer = ::lightning::miao::SetupOptimizer<6, 3>(opt_config);

  info_motion.setIdentity();
  info_motion.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (motion_trans_noise * motion_trans_noise);
  info_motion.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (motion_rot_noise * motion_rot_noise);

  info_loops.setIdentity();
  info_loops.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (loop_trans_noise * loop_trans_noise);
  info_loops.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (loop_rot_noise * loop_rot_noise);

  on_new_frame_id =
    Callbacks::on_new_frame.add(std::bind(&OptimizationMIAO::on_new_frame, this, std::placeholders::_1));

  kill_switch = false;
  input_thread_obj = std::thread(&OptimizationMIAO::input_thread, this);
  worker_thread = std::thread(&OptimizationMIAO::worker_loop, this);
  publish_thread = std::thread(&OptimizationMIAO::publish_loop, this);

  logger->info("OptimizationMIAO initialized");
}

OptimizationMIAO::~OptimizationMIAO() {
  stop();
  Callbacks::on_new_frame.remove(on_new_frame_id);
}

void OptimizationMIAO::stop() {
  if (!kill_switch.exchange(true)) {
    queue_cv.notify_all();
  }
  if (input_thread_obj.joinable()) input_thread_obj.join();
  if (worker_thread.joinable()) worker_thread.join();
  if (publish_thread.joinable()) publish_thread.join();
}

void OptimizationMIAO::on_new_frame(const KeyFrame::ConstPtr& frame) {
  if (!frame->cloud_imu) return;

  const Eigen::Isometry3d& T = frame->T_world_imu;
  Sophus::SO3d rot(T.rotation());

  InputFrame input;
  input.id = frame->id;
  input.stamp = frame->stamp;
  input.pose = SE3d(rot, T.translation());
  input.cloud = frame->cloud_imu;

  std::lock_guard<std::mutex> lock(queue_mutex);
  if (queue.full()) {
    if (++dropped_items % 100 == 1) {
      logger->warn("optimization_miao queue overflow, dropping oldest keyframes ({} dropped)", dropped_items);
    }
  }
  queue.push_back(std::move(input));
  queue_cv.notify_one();
}

void OptimizationMIAO::input_thread() {
  while (true) {
    InputFrame frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !queue.empty() || kill_switch; });
      if (kill_switch && queue.empty()) return;
      frame = queue.front();
      queue.pop_front();
    }

    if (!frame.cloud) continue;
    pcl::PointCloud<PointType>::Ptr cloud_copy(new pcl::PointCloud<PointType>(*frame.cloud));
    add_keyframe(frame.id, frame.stamp, frame.pose, cloud_copy);
  }
}

void OptimizationMIAO::publish_loop() {
  const auto period = std::chrono::microseconds(int(1e6 / std::max(1e-6, publish_interval)));
  while (!kill_switch) {
    std::this_thread::sleep_for(period);
    if (kill_switch) break;

    double latest_stamp = 0.0;
    SE3d latest_pose;
    bool has_pose = get_latest_optimized_pose(latest_stamp, latest_pose);

    if (has_pose) {
      Eigen::Quaterniond q = latest_pose.so3().unit_quaternion();

      // Optimized poses are not keyframes: no id/bias/cloud, matching the
      // frontend KeyFrame semantics loosely enough for trajectory output.
      auto frame = std::make_shared<KeyFrame>();
      frame->id = -1;
      frame->stamp = latest_stamp;
      frame->frame_id = FrameId::IMU;
      frame->T_world_imu = Eigen::Isometry3d::Identity();
      frame->T_world_imu.linear() = q.toRotationMatrix();
      frame->T_world_imu.translation() = latest_pose.translation();
      Callbacks::on_odometry_opt(frame);
    }
  }
}

void OptimizationMIAO::add_keyframe(
  long id,
  double stamp,
  const SE3d& lio_pose,
  const pcl::PointCloud<PointType>::Ptr& cloud) {
  auto kf = std::make_shared<BackendKeyframe>();
  kf->id = id;
  kf->stamp = stamp;
  kf->lio_pose = lio_pose;
  kf->opt_pose = lio_pose;
  // Takes ownership: the caller hands over its (already private) copy.
  kf->cloud = cloud ? cloud : pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

  if (last_kf) {
    SE3d delta = last_kf->lio_pose.inverse() * kf->lio_pose;
    kf->opt_pose = last_kf->opt_pose * delta;
  }

  std::lock_guard<std::mutex> lock(mtx_queue);
  kf_queue.push(kf);
}

void OptimizationMIAO::worker_loop() {
  while (!kill_switch) {
    BackendKeyframe::Ptr kf{nullptr};
    {
      std::lock_guard<std::mutex> lock(mtx_queue);
      if (!kf_queue.empty()) {
        kf = kf_queue.front();
        kf_queue.pop();
      }
    }
    if (kf) {
      handle_kf(kf);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void OptimizationMIAO::handle_kf(const BackendKeyframe::Ptr& kf) {
  if (last_kf && kf->id == last_kf->id) {
    return;
  }

  cur_kf = kf;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    all_keyframes.emplace_back(kf);
  }

  detect_loop_candidates();

  if (verbose) {
    logger->info("lc: get kf {} candi: {}", cur_kf->id, candidates.size());
  }

  compute_loop_candidates();

  pose_optimization();

  last_kf = kf;
}

void OptimizationMIAO::detect_loop_candidates() {
  candidates.clear();

  std::vector<BackendKeyframe::Ptr> kfs_mapping;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    kfs_mapping = all_keyframes;
  }
  BackendKeyframe::Ptr check_first = nullptr;

  if (last_loop_kf == nullptr) {
    last_loop_kf = cur_kf;
    return;
  }

  if (last_loop_kf && (cur_kf->id - last_loop_kf->id) <= loop_kf_gap) {
    logger->info("skip because last loop kf: {}", last_loop_kf->id);
    return;
  }

  for (auto& kf : kfs_mapping) {
    if (check_first != nullptr && std::abs(int(kf->id - check_first->id)) <= min_id_interval) {
      continue;
    }

    if (std::abs(int(kf->id - cur_kf->id)) < closest_id_th) {
      break;
    }

    Vec3d dt = kf->opt_pose.translation() - cur_kf->opt_pose.translation();
    double t2d = dt.head<2>().norm();
    double range_th = max_range;

    if (t2d < range_th) {
      LoopCandidate c(kf->id, cur_kf->id);
      c.tij = kf->lio_pose.inverse() * cur_kf->lio_pose;
      candidates.emplace_back(c);
      check_first = kf;
    }
  }

  if (!candidates.empty()) {
    last_loop_kf = cur_kf;
  }

  if (verbose && !candidates.empty()) {
    logger->info("lc candi: {}", candidates.size());
  }
}

void OptimizationMIAO::compute_loop_candidates() {
  if (candidates.empty()) {
    return;
  }

  std::for_each(candidates.begin(), candidates.end(), [this](LoopCandidate& c) { compute_for_candidate(c); });

  std::vector<LoopCandidate> succ_candidates;
  for (const auto& lc : candidates) {
    if (lc.ndt_score > ndt_score_th) {
      succ_candidates.emplace_back(lc);
    }
  }

  if (verbose) {
    logger->info("success: {}/{}", succ_candidates.size(), candidates.size());
  }

  candidates.swap(succ_candidates);
}

void OptimizationMIAO::compute_for_candidate(LoopCandidate& c) {
  BackendKeyframe::Ptr kf1{nullptr};
  BackendKeyframe::Ptr kf2{nullptr};
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (c.idx1 >= all_keyframes.size() || c.idx2 >= all_keyframes.size()) {
      c.ndt_score = 0;
      return;
    }
    kf1 = all_keyframes[c.idx1];
    kf2 = all_keyframes[c.idx2];
  }

  auto submap_kf1 = build_submap(kf1->id, true);

  pcl::PointCloud<PointType>::Ptr submap_kf2 = kf2->cloud;

  if (submap_kf1->empty() || submap_kf2->empty()) {
    c.ndt_score = 0;
    return;
  }

  Eigen::Matrix4f tw2 = kf2->opt_pose.matrix().cast<float>();

  pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
  std::vector<double> res{10.0, 5.0, 2.0, 1.0};

  pcl::PointCloud<PointType>::Ptr rough_map_1;
  pcl::PointCloud<PointType>::Ptr rough_map_2;

  for (auto& r : res) {
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(40);
    ndt.setResolution(r);
    rough_map_1 = voxel_grid(submap_kf1, r * 0.1);
    rough_map_2 = voxel_grid(submap_kf2, r * 0.1);
    ndt.setInputTarget(rough_map_1);
    ndt.setInputSource(rough_map_2);
    ndt.align(*output, tw2);
    tw2 = ndt.getFinalTransformation();
    c.ndt_score = ndt.getTransformationProbability();
  }

  Eigen::Matrix4d t_mat = tw2.cast<double>();
  Eigen::Quaterniond q(t_mat.block<3, 3>(0, 0));
  q.normalize();
  Vec3d t = t_mat.block<3, 1>(0, 3);

  c.tij = kf1->opt_pose.inverse() * SE3d(Sophus::SO3d(q), t);
}

pcl::PointCloud<PointType>::Ptr OptimizationMIAO::build_submap(int given_id, bool in_world) const {
  pcl::PointCloud<PointType>::Ptr submap(new pcl::PointCloud<PointType>);

  std::vector<BackendKeyframe::Ptr> kfs;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    kfs = all_keyframes;
  }

  SE3d t_ref;
  bool has_ref = false;
  if (!in_world) {
    for (const auto& kf : kfs) {
      if (kf->id == given_id) {
        t_ref = kf->opt_pose.inverse();
        has_ref = true;
        break;
      }
    }
  }

  for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
    int id = idx + given_id;
    if (id < 0 || id >= static_cast<int>(kfs.size())) {
      continue;
    }
    if (id >= static_cast<int>(kfs.size()) || kfs[id]->id != id) {
      continue;
    }

    auto kf = kfs[id];
    pcl::PointCloud<PointType>::Ptr cloud = kf->cloud;
    if (cloud->empty()) {
      continue;
    }

    SE3d twb = kf->opt_pose;
    if (!in_world && has_ref) {
      twb = t_ref * twb;
    }

    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud, *cloud_trans, twb.matrix());

    *submap += *cloud_trans;
  }
  return submap;
}

pcl::PointCloud<PointType>::Ptr OptimizationMIAO::voxel_grid(const pcl::PointCloud<PointType>::Ptr& cloud, float size)
  const {
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(size, size, size);
  voxel.setInputCloud(cloud);
  pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
  voxel.filter(*output);
  return output;
}

void OptimizationMIAO::pose_optimization() {
  auto v = std::make_shared<::lightning::miao::VertexSE3>();
  v->SetId(static_cast<int>(cur_kf->id));
  v->SetEstimate(cur_kf->opt_pose);

  optimizer->AddVertex(v);
  kf_vert.emplace_back(v);

  for (int i = 1; i < 3; i++) {
    int id = static_cast<int>(cur_kf->id) - i;
    if (id >= 0) {
      std::vector<BackendKeyframe::Ptr> kfs;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        kfs = all_keyframes;
      }
      if (id >= static_cast<int>(kfs.size()) || kfs[id]->id != id) {
        continue;
      }
      auto last_kf_local = kfs[id];
      auto last_vert = optimizer->GetVertex(static_cast<int>(last_kf_local->id));
      if (last_vert == nullptr) {
        continue;
      }
      auto e = std::make_shared<::lightning::miao::EdgeSE3>();
      e->SetVertex(0, last_vert);
      e->SetVertex(1, v);
      SE3d motion = last_kf_local->lio_pose.inverse() * cur_kf->lio_pose;
      e->SetMeasurement(motion);
      e->SetInformation(info_motion);
      optimizer->AddEdge(e);
    }
  }

  if (with_height) {
    auto e = std::make_shared<::lightning::miao::EdgeHeightPrior>();
    e->SetVertex(0, v);
    e->SetMeasurement(0);
    e->SetInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 / (height_noise * height_noise));
    optimizer->AddEdge(e);
  }

  for (auto& c : candidates) {
    auto v1 = optimizer->GetVertex(static_cast<int>(c.idx1));
    auto v2 = optimizer->GetVertex(static_cast<int>(c.idx2));
    if (v1 == nullptr || v2 == nullptr) {
      continue;
    }
    auto e = std::make_shared<::lightning::miao::EdgeSE3>();
    e->SetVertex(0, v1);
    e->SetVertex(1, v2);
    e->SetMeasurement(c.tij);
    e->SetInformation(info_loops);

    auto rk = std::make_shared<::lightning::miao::RobustKernelCauchy>();
    rk->SetDelta(rk_loop_th);
    e->SetRobustKernel(rk);

    optimizer->AddEdge(e);
    edge_loops.emplace_back(e);
  }

  if (optimizer->GetEdges().empty()) {
    return;
  }

  if (candidates.empty()) {
    return;
  }

  optimizer->InitializeOptimization();
  optimizer->SetVerbose(false);

  optimizer->Optimize(20);

  int cnt_outliers = 0;
  for (auto& e : edge_loops) {
    if (e->GetRobustKernel() == nullptr) {
      continue;
    }
    if (e->Chi2() > e->GetRobustKernel()->Delta()) {
      e->SetLevel(1);
      cnt_outliers++;
    } else {
      e->SetRobustKernel(nullptr);
    }
  }

  if (verbose) {
    logger->info("loop outliers: {}/{}", cnt_outliers, edge_loops.size());
  }

  std::vector<BackendKeyframe::Ptr> kfs;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    kfs = all_keyframes;
  }
  for (auto& vert : kf_vert) {
    SE3d pose = vert->Estimate();
    int vid = vert->GetId();
    if (vid >= 0 && vid < static_cast<int>(kfs.size()) && kfs[vid]->id == vid) {
      kfs[vid]->opt_pose = pose;
    }
  }

  logger->info("optimize finished, loops: {}", edge_loops.size());
}

bool OptimizationMIAO::get_latest_optimized_pose(double& stamp, SE3d& pose) const {
  std::lock_guard<std::mutex> lock(state_mutex);
  if (all_keyframes.empty()) {
    return false;
  }
  const auto& kf = all_keyframes.back();
  stamp = kf->stamp;
  pose = kf->opt_pose;
  return true;
}

}  // namespace optimization_miao
}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::optimization_miao::OptimizationMIAO();
}
