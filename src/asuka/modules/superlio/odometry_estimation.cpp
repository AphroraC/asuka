#include "asuka/modules/superlio/odometry_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <pcl/common/transforms.h>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace superlio {

namespace {

struct ThreadAcc {
  Eigen::Matrix<double, 6, 6> htvh = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> htvr = Eigen::Matrix<double, 6, 1>::Zero();
  ThreadAcc() : htvh(Eigen::Matrix<double, 6, 6>::Zero()), htvr(Eigen::Matrix<double, 6, 1>::Zero()) {}
};

}  // namespace

bool OdometryEstimation::calc_plane_coeff(int n, const std::array<Eigen::Matrix<float, 3, 1>, 5>& points, std::array<double, 4>& abcd) {
  Eigen::Vector3d normvec;
  if (n == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  } else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < n; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double nn = normvec.norm();
  if (nn < 1e-6f) return false;

  abcd[3] = 1.0 / nn;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];

  for (int i = 0; i < n; ++i) {
    const Eigen::Matrix<float, 3, 1>& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}

bool OdometryEstimation::compute_error(
  const std::array<double, 4>& abcd,
  const Eigen::Matrix<float, 3, 1>& point,
  float length,
  float& error) {
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config config = load_config("config_odometry");
  param.kf_max_iterations = config.param<int>("kf", "max_iterations", 4);
  param.kf_align_gravity = config.param<bool>("kf", "align_gravity", true);
  param.kf_quit_eps = config.param<double>("kf", "quit_eps", 0.001);
  param.imu_na = config.param<double>("imu", "imu_na", 0.1);
  param.imu_ng = config.param<double>("imu", "imu_ng", 0.1);
  param.imu_nba = config.param<double>("imu", "imu_nba", 0.0001);
  param.imu_nbg = config.param<double>("imu", "imu_nbg", 0.0001);
  param.gravity_norm = config.param<double>("imu", "gravity_norm", 9.7946);
  param.ivox_capacity = static_cast<std::size_t>(config.param<int>("hash_map", "hash_capacity", 2000000));
  param.ivox_resolution = static_cast<float>(config.param<double>("hash_map", "vox_resolution", 0.5));
  param.voxel_filter_size = static_cast<float>(config.param<double>("preprocess", "voxel_filter_size", 0.5));
  param.enable_downsample = config.param<bool>("preprocess", "enable_downsample", true);
  param.time_eva = config.param<bool>("eva", "timer", false);

  param.lidar_buffer_capacity = config.param<int>("odometry", "lidar_buffer_capacity", 50);
  param.imu_buffer_capacity = config.param<int>("odometry", "imu_buffer_capacity", 1000);
  lidar_buffer.set_capacity(param.lidar_buffer_capacity);
  measures.imu.set_capacity(param.imu_buffer_capacity);

  const std::vector<double> odom =
    config.param<std::vector<double>>("extrinsic", "odom_robo", std::vector<double>(6, 0.0));
  Eigen::Matrix<float, 3, 1> odom_t(static_cast<float>(odom[0]), static_cast<float>(odom[1]), static_cast<float>(odom[2]));
  Eigen::Matrix3d temp_r = (Eigen::AngleAxisd(odom[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(odom[4] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(odom[3] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
  Eigen::Matrix<float, 3, 3> odom_r = temp_r.transpose().cast<float>();
  param.odom_robo = Se3(odom_r, odom_t);
  param.lidar_robo_yaw =
    Eigen::AngleAxisd(odom[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<float>();

  const Config ros_config = load_config("config_ros");
  if (!config.has("sensor")) {
    throw std::runtime_error("config_odometry: no sensor block");
  }
  Eigen::Vector3d t_ext =
    config.param<Eigen::Vector3d>("sensor", "extrinsic_T", Eigen::Vector3d(-0.011, -0.02329, 0.04412));
  Eigen::Matrix3d r_ext = config.param<Eigen::Matrix3d>("sensor", "extrinsic_R", Eigen::Matrix3d::Identity());
  Eigen::Matrix<float, 3, 3> r_lidar_imu = r_ext.cast<float>();
  Eigen::Matrix<float, 3, 1> t_lidar_imu = t_ext.cast<float>();
  param.lidar_imu = Se3(r_lidar_imu, t_lidar_imu);

  param.save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);

  ivox = std::make_shared<OctVoxMapType>(typename OctVoxMapType::Options(param.ivox_resolution, param.ivox_capacity));
  kf = std::make_shared<Eskf>();
  voxel_grid_filter.set_leaf_size(param.voxel_filter_size);

  scan_undistort_full.reset(new PointCloudT());
  ds_undistort.reset(new PointCloudT());
  world_pc.reset(new PointCloudT());
  map_cloud.reset(new PointCloudT());
  points_world.reserve(21000);
  abcd_vec.resize(20000);
  effect_knn_idxs.resize(20000);

  state_fn = &OdometryEstimation::state_wait_kf_init;

  logger->info(
    "superlio::OdometryEstimation initialized: gravity={} kf_iter={} ivox(cap={},res={}) voxel={}",
    param.gravity_norm,
    param.kf_max_iterations,
    param.ivox_capacity,
    param.ivox_resolution,
    param.voxel_filter_size);
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

void OdometryEstimation::stop() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  imu_preprocess->imu_buffer.clear();
  lidar_pushed = false;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return map_cloud;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  imu_preprocess->insert_imu(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  LidarData lidar_data;
  lidar_data.start_time = stamp;
  lidar_data.end_time = stamp;
  if (cloud && !cloud->empty()) {
    lidar_data.end_time = stamp + cloud->points.back().curvature / 1000.0;
  }
  lidar_data.pc = cloud;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    lidar_buffer.push_back(lidar_data);
  }
}

int OdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return static_cast<int>(lidar_buffer.size());
}

bool OdometryEstimation::process_once() {
  if (!sync_packages(measures)) return false;
  (this->*state_fn)();
  return true;
}

bool OdometryEstimation::sync_packages(MeasureGroup& meas) {
  std::lock_guard<std::mutex> lock(buffer_mutex);

  if (lidar_buffer.empty() || imu_preprocess->imu_buffer.empty()) {
    return false;
  }

  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    lidar_pushed = true;
  }

  if (last_timestamp_lidar > meas.lidar.end_time) {
    lidar_buffer.pop_front();
    lidar_pushed = false;
    return false;
  }

  if (imu_preprocess->last_timestamp < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_preprocess->imu_buffer.front().secs;
  meas.imu.clear();
  while ((!imu_preprocess->imu_buffer.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_preprocess->imu_buffer.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_preprocess->imu_buffer.front());
    imu_preprocess->imu_buffer.pop_front();
  }

  last_timestamp_lidar = meas.lidar.end_time;
  lidar_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void OdometryEstimation::state_wait_kf_init() {
  if (kf_init()) {
    state_fn = &OdometryEstimation::state_wait_map_init;
    logger->info("---> [SuperLIO]: KF init done");
  }
}

void OdometryEstimation::state_wait_map_init() {
  if (map_init()) {
    kf->inited = true;
    state_fn = &OdometryEstimation::state_process;
    logger->info("---> [SuperLIO]: Map init done");
  }
}

void OdometryEstimation::state_process() {
  frame_num++;
  if (param.time_eva) {
    time_record.evaluate([this]() { propagation_undistort(); }, "[Undistort]");
    time_record.evaluate([this]() { down_sample(); }, "[DownSample]");
    time_record.evaluate([this]() { observe(); }, "[Observe]");
    time_record.evaluate([this]() { update_map(); }, "[UpdateMap]");
  } else {
    propagation_undistort();
    down_sample();
    observe();
    update_map();
  }
  output();
}

bool OdometryEstimation::kf_init() {
  for (auto& imu : measures.imu) {
    imu_preprocess->accumulate(imu);
  }

  if (imu_preprocess->imu_count < 50) {
    return false;
  }

  Eigen::Matrix<float, 3, 1> mean_gyro = imu_preprocess->mean_gyro;
  Eigen::Matrix<float, 3, 1> mean_acce = imu_preprocess->mean_acc;

  Eigen::Matrix<float, 3, 1> gravity = -mean_acce * param.gravity_norm / mean_acce.norm();
  Eigen::Matrix<float, 3, 1> ref_gravity(0, 0, -static_cast<float>(param.gravity_norm));
  Eigen::Matrix<float, 3, 3> init_rot = Eigen::Quaternion<float>::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  Eigen::Matrix<float, 3, 1> n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  Eigen::Matrix<float, 3, 3> r_yaw_inv = Eigen::AngleAxis<float>(-static_cast<float>(yaw), Eigen::Matrix<float, 3, 1>::UnitZ()).toRotationMatrix();
  Eigen::Matrix<float, 3, 3> rot = param.lidar_robo_yaw * r_yaw_inv * init_rot;

  Eskf::Options options;
  options.gyro_var = param.imu_ng;
  options.acce_var = param.imu_na;
  options.bias_gyro_var = param.imu_nbg;
  options.bias_acce_var = param.imu_nba;
  options.num_iterations = param.kf_max_iterations;
  options.quit_eps = param.kf_quit_eps;

  float imu_scale = static_cast<float>(param.gravity_norm / mean_acce.norm());
  kf->set_initial_conditions(options, mean_gyro, Eigen::Matrix<float, 3, 1>::Zero(), imu_scale, ref_gravity);
  auto state = kf->get_sys_state();
  state.rot = So3(rot);
  state.p = param.odom_robo.t;
  state.timestamp = measures.imu.back().secs;
  kf->set_x(state);
  sys_init_pose = kf->get_se3();
  return true;
}

bool OdometryEstimation::map_init() {
  frame_num++;

  std::size_t ptsize = measures.lidar.pc->size();
  points_world.resize(ptsize);

  const Se3 transform = sys_init_pose * param.lidar_imu;

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
      auto& point_pcl = measures.lidar.pc->points[idx];
      Eigen::Matrix<float, 3, 1> point_body(point_pcl.x, point_pcl.y, point_pcl.z);
      points_world[idx] = transform * point_body;
    }
  });

  ivox->insert(points_world);
  kf->set_last_obs_time(measures.lidar.end_time);

  if (frame_num > 3) {
    return true;
  }
  return false;
}

void OdometryEstimation::propagation_undistort() {
  propagate_states.clear();
  propagate_states.emplace_back(kf->get_dynamic_state());
  kf->set_obs_time(measures.lidar.end_time);
  for (auto& imu : measures.imu) {
    kf->predict(imu);
    propagate_states.emplace_back(kf->get_dynamic_state());
  }

  const Eigen::Matrix<float, 3, 3> tli_r = param.lidar_imu.r;
  const Eigen::Matrix<float, 3, 1> tli_t = param.lidar_imu.t;
  const Se3 t_end = kf->get_se3();
  const Eigen::Matrix<float, 3, 3> r_inv = t_end.r.transpose();
  const Eigen::Matrix<float, 3, 1> t_end_t = t_end.t;
  const double start_time = measures.lidar.start_time;
  auto& raw_pc = measures.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full->resize(ptsize);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    Eigen::Matrix<float, 3, 3> r_h, r_t;
    Eigen::Matrix<float, 3, 1> p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
      auto& pt_full = scan_undistort_full->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.curvature / 1000.0;
      if (query_time > propagate_states.back().time) {
        // Beyond the propagated states: keep the point in
        // the same end-time body frame as the normal
        // branch. Upstream Super-LIO skips the back
        // rotation here, mixing query-time and end-time
        // frames.
        Eigen::Matrix<float, 3, 1> raw(pt.x, pt.y, pt.z);
        Eigen::Matrix<float, 3, 1> eigen_point = r_inv * (tli_r * raw + tli_t);
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = propagate_states.begin();
      for (auto iter = propagate_states.begin(); iter != propagate_states.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double tau = query_time - match_iter->time;
      double s = tau / dt;
      r_h = match_iter->rot;
      r_t = match_iter_n->rot;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      Eigen::Matrix<float, 3, 3> r_i = Eigen::Quaternion<float>(r_h).slerp(s, Eigen::Quaternion<float>(r_t)).toRotationMatrix();
      Eigen::Matrix<float, 3, 1> p_i = p_h + v_h * tau + 0.5 * acc_t * tau * tau;
      Eigen::Matrix<float, 3, 1> t_ei = p_i - t_end_t;
      Eigen::Matrix<float, 3, 1> raw(pt.x, pt.y, pt.z);
      Eigen::Matrix<float, 3, 1> eigen_point = r_inv * (r_i * (tli_r * raw + tli_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}

void OdometryEstimation::down_sample() {
  voxel_grid_filter.set_input_cloud(scan_undistort_full);
  voxel_grid_filter.filter(ds_undistort);
}

void OdometryEstimation::observe() {
  std::size_t ptsize = ds_undistort->size();

  std::vector<float> lengths;
  points_body.resize(ptsize);
  lengths.resize(ptsize);

  effect_knn_num = ptsize;
  std::iota(effect_knn_idxs.begin(), effect_knn_idxs.begin() + ptsize, 0);

  for (std::size_t i = 0; i < ptsize; ++i) {
    const auto& point_body_pcl = ds_undistort->points[i];
    points_body[i] = Eigen::Matrix<float, 3, 1>(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    lengths[i] = points_body[i].norm();
  }

  ivox->reset_max_group();
  int iter_num = 0;

  kf->update_observe([&, this](const Eskf::KfState& kf_state, Eigen::Matrix<float, 6, 6>& htvh, Eigen::Matrix<float, 6, 1>& htvr) {
    const Se3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const Eigen::Matrix<double, 3, 3> r_transpose = (pose.r.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadAcc> tls_acc;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, effect_knn_num), [&](const tbb::blocked_range<size_t>& r) {
      KnnHeapType top_k;
      auto& local_acc = tls_acc.local();
      for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
        int idx = effect_knn_idxs[r_s];
        Eigen::Matrix<float, 3, 1>& point_body = points_body[idx];
        Eigen::Matrix<float, 3, 1> point_world = pose * point_body;

        if (!need_converge) {
          top_k.reset();
          ivox->get_top_k(point_world, top_k);
          if (top_k.count < 4) {
            effect_mask[idx] = false;
            effect_knn_mask[idx] = false;
            continue;
          }
          effect_knn_mask[idx] = true;
          effect_mask[idx] = calc_plane_coeff(top_k.count, top_k.points, abcd_vec[idx]);
        }

        if (!effect_mask[idx]) continue;

        auto& abcd = abcd_vec[idx];
        float error;
        effect_mask[idx] = compute_error(abcd, point_world, lengths[idx], error);
        if (!effect_mask[idx]) continue;

        {
          Eigen::Matrix<double, 3, 1> normvec(abcd[0], abcd[1], abcd[2]);
          Eigen::Matrix<double, 3, 1> nb = r_transpose * normvec;
          Eigen::Matrix<double, 3, 1> point_body_d = point_body.cast<double>();
          Eigen::Matrix<double, 6, 1> j;
          j.head<3>() = point_body_d.cross(nb);
          j.tail<3>() = normvec;

          local_acc.htvh += j * 1000 * j.transpose();
          local_acc.htvr -= j * 1000 * error;
        }
      }
    });

    Eigen::Matrix<double, 6, 6> sum_htvh = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> sum_htvr = Eigen::Matrix<double, 6, 1>::Zero();
    for (const auto& local_acc : tls_acc) {
      sum_htvh += local_acc.htvh;
      sum_htvr += local_acc.htvr;
    }
    htvh = sum_htvh.cast<float>();
    htvr = sum_htvr.cast<float>();

    if (need_converge) return;

    int local_effect_knn_num = 0;
    for (std::size_t i = 0; i < effect_knn_num; ++i) {
      int idx = effect_knn_idxs[i];
      if (!effect_knn_mask[idx]) continue;
      effect_knn_idxs[local_effect_knn_num] = idx;
      local_effect_knn_num++;
    }

    effect_knn_num = local_effect_knn_num;

    iter_num++;
  });

  frame_num++;
}

void OdometryEstimation::update_map() {
  const std::size_t ptsize = ds_undistort->size();
  if (ptsize == 0) return;

  last_pose = kf->get_se3();
  points_world.resize(ptsize);

  const auto r = last_pose.r;
  const auto t = last_pose.t;

  for (std::size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body[i];
    points_world[i] = r * pt + t;
  }

  ivox->insert(points_world);
}

void OdometryEstimation::output() {
  NavState state = kf->get_nav_state();

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.rot.r.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  pcl::transformPointCloud(*ds_undistort, *world_pc, transformation);

  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  T_world_imu.linear() = state.rot.r.cast<double>();
  T_world_imu.translation() = state.p.cast<double>();

  auto frame = std::make_shared<KeyFrame>();
  frame->id = frame_counter++;
    // KeyFrame::stamp contract: sweep start (see callbacks.hpp on_new_frame).
  frame->stamp = measures.lidar.start_time;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = T_world_imu;
  frame->v_world_imu = state.v.cast<double>();
  SysState sys_state = kf->get_sys_state();
  frame->imu_bias.head<3>() = sys_state.bg.cast<double>();
  frame->imu_bias.tail<3>() = sys_state.ba.cast<double>();

  // Hand the frame the undistorted scan itself, no copy: the shared_ptr
  // transfers ownership to the observers, and a fresh buffer is allocated for
  // the next scan (down_sample() refills it before anything reads it again).
  const std::size_t ds_size = ds_undistort->size();
  frame->cloud_imu = ds_undistort;
  ds_undistort.reset(new PointCloudT());

  Callbacks::on_new_frame(frame);

  if (param.save_map_en) {
    *map_cloud += *world_pc;
  }

  logger->debug(
    "[superlio] scan stamp={:.3f} ds={} effect_knn={} map_size={}",
    measures.lidar.start_time,
    ds_size,
    effect_knn_num,
    ivox ? 0 : 0);
}

}  // namespace superlio
}  // namespace asuka
