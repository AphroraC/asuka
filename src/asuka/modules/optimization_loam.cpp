#include <asuka/modules/optimization_loam.hpp>

#include <chrono>
#include <cmath>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace optimization_loam {

gtsam::Pose3 pose6d_to_gtsam(const Pose6D& p) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

Pose6D diff_transformation(const Pose6D& p1, const Pose6D& p2) {
  Eigen::Affine3f se3_p1 = pcl::getTransformation(p1.x, p1.y, p1.z, p1.roll, p1.pitch, p1.yaw);
  Eigen::Affine3f se3_p2 = pcl::getTransformation(p2.x, p2.y, p2.z, p2.roll, p2.pitch, p2.yaw);
  Eigen::Matrix4f se3_delta0 = se3_p1.matrix().inverse() * se3_p2.matrix();
  Eigen::Affine3f se3_delta;
  se3_delta.matrix() = se3_delta0;
  float dx, dy, dz, droll, dpitch, dyaw;
  pcl::getTranslationAndEulerAngles(se3_delta, dx, dy, dz, droll, dpitch, dyaw);
  return Pose6D{
    double(std::abs(dx)),
    double(std::abs(dy)),
    double(std::abs(dz)),
    double(std::abs(droll)),
    double(std::abs(dpitch)),
    double(std::abs(dyaw))};
}

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloud_in, const Pose6D& tf) {
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());
  int cloud_size = cloud_in->size();
  cloud_out->resize(cloud_size);
  Eigen::Affine3f trans_cur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
  int number_of_cores = 16;
#pragma omp parallel for num_threads(number_of_cores)
  for (int i = 0; i < cloud_size; ++i) {
    const auto& point_from = cloud_in->points[i];
    cloud_out->points[i].x = trans_cur(0, 0) * point_from.x + trans_cur(0, 1) * point_from.y +
                             trans_cur(0, 2) * point_from.z + trans_cur(0, 3);
    cloud_out->points[i].y = trans_cur(1, 0) * point_from.x + trans_cur(1, 1) * point_from.y +
                             trans_cur(1, 2) * point_from.z + trans_cur(1, 3);
    cloud_out->points[i].z = trans_cur(2, 0) * point_from.x + trans_cur(2, 1) * point_from.y +
                             trans_cur(2, 2) * point_from.z + trans_cur(2, 3);
    cloud_out->points[i].intensity = point_from.intensity;
  }
  return cloud_out;
}

Eigen::Affine3f pose6d_to_affine(const Pose6D& pose) {
  return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

OptimizationLOAM::OptimizationLOAM() {
  logger = create_module_logger("loam");

  const Config config(GlobalConfig::get_config_path("config_extensions"));
  keyframe_meter_gap = config.param<double>("optimization_loam", "keyframe_meter_gap", 2.0);
  keyframe_deg_gap = config.param<double>("optimization_loam", "keyframe_deg_gap", 10.0);
  keyframe_rad_gap = keyframe_deg_gap * M_PI / 180.0;
  sc_dist_thres = config.param<double>("optimization_loam", "sc_dist_thres", 0.2);
  sc_max_radius = config.param<double>("optimization_loam", "sc_max_radius", 80.0);
  history_keyframe_search_radius = config.param<double>("optimization_loam", "history_keyframe_search_radius", 10.0);
  history_keyframe_search_time_diff =
    config.param<double>("optimization_loam", "history_keyframe_search_time_diff", 30.0);
  history_keyframe_search_num = config.param<int>("optimization_loam", "history_keyframe_search_num", 25);
  loop_noise_score = config.param<double>("optimization_loam", "loop_noise_score", 0.5);
  graph_update_times = config.param<int>("optimization_loam", "graph_update_times", 2);
  loop_fitness_score_threshold = config.param<double>("optimization_loam", "loop_fitness_score_threshold", 0.3);
  loop_closure_frequency = config.param<double>("optimization_loam", "loop_closure_frequency", 2.0);
  graph_update_frequency = config.param<double>("optimization_loam", "graph_update_frequency", 1.0);
  vizmap_frequency = config.param<double>("optimization_loam", "vizmap_frequency", 0.1);
  publish_interval = 1.0 / config.param<double>("optimization_loam", "publish_rate", 10.0);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = std::make_unique<gtsam::ISAM2>(parameters);
  init_noises();

  sc_manager.setSCdistThres(sc_dist_thres);
  sc_manager.setMaximumRadius(sc_max_radius);

  const float filter_size = 0.4f;
  downsize_filter_scancontext.setLeafSize(filter_size, filter_size, filter_size);
  downsize_filter_icp.setLeafSize(filter_size, filter_size, filter_size);
  double map_viz_filter_size = config.param<double>("optimization_loam", "mapviz_filter_size", 0.4);
  downsize_filter_map_pgo.setLeafSize(map_viz_filter_size, map_viz_filter_size, map_viz_filter_size);

  on_new_frame_id =
    Callbacks::on_new_frame.add(std::bind(&OptimizationLOAM::on_new_frame, this, std::placeholders::_1));

  kill_switch = false;
  input_thread_obj = std::thread(&OptimizationLOAM::input_thread, this);
  lcd_thread = std::thread(&OptimizationLOAM::process_lcd, this);
  icp_thread = std::thread(&OptimizationLOAM::process_icp, this);
  isam_thread = std::thread(&OptimizationLOAM::process_isam, this);
  viz_map_thread = std::thread(&OptimizationLOAM::process_viz_map, this);
  publish_thread = std::thread(&OptimizationLOAM::publish_loop, this);

  logger->info("OptimizationLOAM initialized");
}

OptimizationLOAM::~OptimizationLOAM() {
  stop();
  Callbacks::on_new_frame.remove(on_new_frame_id);
}

void OptimizationLOAM::stop() {
  if (!kill_switch.exchange(true)) {
    queue_cv.notify_all();
  }
  if (input_thread_obj.joinable()) input_thread_obj.join();
  if (lcd_thread.joinable()) lcd_thread.join();
  if (icp_thread.joinable()) icp_thread.join();
  if (isam_thread.joinable()) isam_thread.join();
  if (viz_map_thread.joinable()) viz_map_thread.join();
  if (publish_thread.joinable()) publish_thread.join();
}

void OptimizationLOAM::on_new_frame(const KeyFrame::ConstPtr& frame) {
  if (!frame->cloud_imu) return;

  Eigen::Affine3f aff = Eigen::Affine3f::Identity();
  aff.matrix() = frame->T_world_imu.matrix().cast<float>();
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(aff, x, y, z, roll, pitch, yaw);

  InputFrame input;
  input.stamp = frame->stamp;
  input.pose.x = x;
  input.pose.y = y;
  input.pose.z = z;
  input.pose.roll = roll;
  input.pose.pitch = pitch;
  input.pose.yaw = yaw;
  input.cloud = frame->cloud_imu;

  std::lock_guard<std::mutex> lock(queue_mutex);
  if (queue.full()) {
    if (++dropped_items % 100 == 1) {
      logger->warn("optimization_loam queue overflow, dropping oldest keyframes ({} dropped)", dropped_items);
    }
  }
  queue.push_back(std::move(input));
  queue_cv.notify_one();
}

void OptimizationLOAM::input_thread() {
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
    add_keyframe(frame.stamp, frame.pose, cloud_copy);
  }
}

void OptimizationLOAM::publish_loop() {
  const auto period = std::chrono::microseconds(int(1e6 / std::max(1e-6, publish_interval)));
  while (!kill_switch) {
    std::this_thread::sleep_for(period);
    if (kill_switch) break;

    double latest_stamp = 0.0;
    Pose6D latest_pose{};
    bool has_pose = get_latest_pose(latest_stamp, latest_pose);

    if (has_pose) {
      Eigen::Quaterniond q = Eigen::AngleAxisd(latest_pose.yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(latest_pose.pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(latest_pose.roll, Eigen::Vector3d::UnitX());

      // Optimized poses are not keyframes: no id/bias/cloud, matching the
      // frontend KeyFrame semantics loosely enough for trajectory output.
      auto frame = std::make_shared<KeyFrame>();
      frame->id = -1;
      frame->stamp = latest_stamp;
      frame->frame_id = FrameId::IMU;
      frame->T_world_imu = Eigen::Isometry3d::Identity();
      frame->T_world_imu.linear() = q.toRotationMatrix();
      frame->T_world_imu.translation() = Eigen::Vector3d(latest_pose.x, latest_pose.y, latest_pose.z);
      Callbacks::on_odometry_opt(frame);
    }
  }
}

void OptimizationLOAM::add_keyframe(double stamp, const Pose6D& pose, const pcl::PointCloud<PointType>::Ptr& cloud) {
  odom_pose_prev = odom_pose_curr;
  odom_pose_curr = pose;
  Pose6D dtf = diff_transformation(odom_pose_prev, odom_pose_curr);
  double delta_translation = std::sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
  translation_accumulated += delta_translation;
  rotation_accumulated += (dtf.roll + dtf.pitch + dtf.yaw);

  bool is_now_keyframe = false;
  if (translation_accumulated > keyframe_meter_gap || rotation_accumulated > keyframe_rad_gap) {
    is_now_keyframe = true;
    translation_accumulated = 0.0;
    rotation_accumulated = 0.0;
  }
  if (!is_now_keyframe) return;

  pcl::PointCloud<PointType>::Ptr this_keyframe_ds(new pcl::PointCloud<PointType>());
  downsize_filter_scancontext.setInputCloud(cloud);
  downsize_filter_scancontext.filter(*this_keyframe_ds);

  {
    std::lock_guard<std::mutex> lock(m_kf);
    keyframe_laser_clouds.push_back(this_keyframe_ds);
    keyframe_poses.push_back(pose);
    keyframe_poses_updated.push_back(pose);
    keyframe_times.push_back(stamp);
    sc_manager.makeAndSaveScancontextAndKeys(*this_keyframe_ds);
  }

  const int prev_node_idx = keyframe_poses.size() - 2;
  const int curr_node_idx = keyframe_poses.size() - 1;
  if (!gt_sam_graph_made) {
    const int init_node_idx = 0;
    gtsam::Pose3 pose_origin = pose6d_to_gtsam(keyframe_poses.at(init_node_idx));
    {
      std::lock_guard<std::mutex> lock(mtx_posegraph);
      gt_sam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, pose_origin, prior_noise));
      initial_estimate.insert(init_node_idx, pose_origin);
    }
    gt_sam_graph_made = true;
    logger->info("posegraph prior node {} added", init_node_idx);
  } else {
    gtsam::Pose3 pose_from = pose6d_to_gtsam(keyframe_poses.at(prev_node_idx));
    gtsam::Pose3 pose_to = pose6d_to_gtsam(keyframe_poses.at(curr_node_idx));
    {
      std::lock_guard<std::mutex> lock(mtx_posegraph);
      gt_sam_graph.add(
        gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, pose_from.between(pose_to), odom_noise));
      initial_estimate.insert(curr_node_idx, pose_to);
    }
    if (curr_node_idx % 100 == 0) logger->info("posegraph odom node {} added", curr_node_idx);
  }
}

void OptimizationLOAM::init_noises() {
  gtsam::Vector prior_noise_vector6(6);
  prior_noise_vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_noise_vector6);

  gtsam::Vector odom_noise_vector6(6);
  odom_noise_vector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_noise_vector6);

  gtsam::Vector robust_noise_vector6(6);
  robust_noise_vector6 << loop_noise_score, loop_noise_score, loop_noise_score, loop_noise_score, loop_noise_score,
    loop_noise_score;
  robust_loop_noise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
    gtsam::noiseModel::Diagonal::Variances(robust_noise_vector6));
}

void OptimizationLOAM::run_isam2opt() {
  isam->update(gt_sam_graph, initial_estimate);
  isam->update();
  for (int i = graph_update_times; i > 0; --i) isam->update();
  gt_sam_graph.resize(0);
  initial_estimate.clear();
  isam_current_estimate = isam->calculateEstimate();
  update_poses();
}

void OptimizationLOAM::update_poses() {
  std::lock_guard<std::mutex> lock(m_kf);
  for (int node_idx = 0; node_idx < int(isam_current_estimate.size()); node_idx++) {
    Pose6D& p = keyframe_poses_updated[node_idx];
    p.x = isam_current_estimate.at<gtsam::Pose3>(node_idx).translation().x();
    p.y = isam_current_estimate.at<gtsam::Pose3>(node_idx).translation().y();
    p.z = isam_current_estimate.at<gtsam::Pose3>(node_idx).translation().z();
    p.roll = isam_current_estimate.at<gtsam::Pose3>(node_idx).rotation().roll();
    p.pitch = isam_current_estimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
    p.yaw = isam_current_estimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
  }
  recent_idx_updated = int(keyframe_poses_updated.size()) - 1;
}

void OptimizationLOAM::loop_find_near_keyframes(
  pcl::PointCloud<PointType>::Ptr& near_keyframes,
  int key,
  int search_num) {
  near_keyframes->clear();
  int cloud_size = keyframe_laser_clouds.size();
  for (int i = -search_num; i <= search_num; ++i) {
    int key_near = key + i;
    if (key_near < 0 || key_near >= cloud_size) continue;
    std::lock_guard<std::mutex> lock(m_kf);
    *near_keyframes += *local2global(keyframe_laser_clouds[key_near], keyframe_poses_updated[key_near]);
  }
  if (near_keyframes->empty()) return;
  pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
  downsize_filter_icp.setInputCloud(near_keyframes);
  downsize_filter_icp.filter(*cloud_temp);
  *near_keyframes = *cloud_temp;
}

gtsam::Pose3 OptimizationLOAM::do_icp_virtual_relative(int loop_kf_idx, int curr_kf_idx) {
  pcl::PointCloud<PointType>::Ptr cure_keyframe_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr target_keyframe_cloud(new pcl::PointCloud<PointType>());
  loop_find_near_keyframes(cure_keyframe_cloud, curr_kf_idx, 0);
  loop_find_near_keyframes(target_keyframe_cloud, loop_kf_idx, history_keyframe_search_num);

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(150);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(cure_keyframe_cloud);
  icp.setInputTarget(target_keyframe_cloud);
  pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  if (icp.hasConverged() == false || icp.getFitnessScore() > loop_fitness_score_threshold) {
    logger->info(
      "[SC loop] ICP fitness test failed ({} > {}). Reject this SC loop.",
      icp.getFitnessScore(),
      loop_fitness_score_threshold);
    return gtsam::Pose3::Identity();
  }
  logger->info(
    "[SC loop] ICP fitness test passed ({} < {}). Add this SC loop.",
    icp.getFitnessScore(),
    loop_fitness_score_threshold);

  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correction_lidar_frame;
  correction_lidar_frame.matrix() = icp.getFinalTransformation();
  std::lock_guard<std::mutex> lock(m_kf);
  Eigen::Affine3f t_wrong = pose6d_to_affine(keyframe_poses_updated[curr_kf_idx]);
  Eigen::Affine3f t_correct = correction_lidar_frame * t_wrong;
  pcl::getTranslationAndEulerAngles(t_correct, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 pose_from = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
  gtsam::Pose3 pose_to = pose6d_to_gtsam(keyframe_poses_updated[loop_kf_idx]);
  return pose_from.between(pose_to);
}

void OptimizationLOAM::perform_sc_loop_closure() {
  if (int(keyframe_poses.size()) < sc_manager.NUM_EXCLUDE_RECENT) return;
  auto detect_result = sc_manager.detectLoopClosureID();
  int sc_closest_history_frame_id = detect_result.first;
  if (sc_closest_history_frame_id != -1) {
    const int prev_node_idx = sc_closest_history_frame_id;
    const int curr_node_idx = keyframe_poses.size() - 1;
    logger->info("Loop detected! - between {} and {}", prev_node_idx, curr_node_idx);
    std::lock_guard<std::mutex> lock(m_buf);
    sc_loop_icp_buf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
  }
}

bool OptimizationLOAM::detect_loop_closure_distance(int* loop_key_cur, int* loop_key_pre) {
  auto it = loop_index_container.find(*loop_key_cur);
  if (it != loop_index_container.end()) return false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud_key_poses3d(new pcl::PointCloud<pcl::PointXYZ>());
  {
    std::lock_guard<std::mutex> lock(m_kf);
    for (const auto& p : keyframe_poses) copy_cloud_key_poses3d->points.emplace_back(p.x, p.y, p.z);
  }
  std::vector<int> point_search_ind_loop;
  std::vector<float> point_search_sq_dis_loop;
  kdtree_history_key_poses->setInputCloud(copy_cloud_key_poses3d);
  kdtree_history_key_poses->radiusSearch(
    copy_cloud_key_poses3d->back(),
    history_keyframe_search_radius,
    point_search_ind_loop,
    point_search_sq_dis_loop,
    0);
  std::vector<double> times_copy;
  {
    std::lock_guard<std::mutex> lock(m_kf);
    times_copy = keyframe_times;
  }
  for (int i = 0; i < int(point_search_ind_loop.size()); ++i) {
    int id = point_search_ind_loop[i];
    if (std::abs(times_copy[id] - times_copy[*loop_key_cur]) > history_keyframe_search_time_diff) {
      *loop_key_pre = id;
      break;
    }
  }
  if (*loop_key_pre == -1 || *loop_key_cur == *loop_key_pre) return false;
  return true;
}

void OptimizationLOAM::perform_rs_loop_closure() {
  if (keyframe_poses.empty()) return;
  int loop_key_cur = keyframe_poses.size() - 1;
  int loop_key_pre = -1;
  if (detect_loop_closure_distance(&loop_key_cur, &loop_key_pre)) {
    logger->info("Loop detected! - between {} and {}", loop_key_pre, loop_key_cur);
    std::lock_guard<std::mutex> lock(m_buf);
    sc_loop_icp_buf.push(std::pair<int, int>(loop_key_pre, loop_key_cur));
    loop_index_container[loop_key_cur] = loop_key_pre;
  }
}

void OptimizationLOAM::process_lcd() {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::microseconds(int(1e6 / std::max(1e-6, loop_closure_frequency))));
    if (kill_switch) break;
    {
      std::lock_guard<std::mutex> lock(m_kf);
      if (keyframe_poses.empty()) continue;
    }
    perform_sc_loop_closure();
    perform_rs_loop_closure();
  }
}

void OptimizationLOAM::process_icp() {
  while (!kill_switch) {
    bool empty = false;
    {
      std::lock_guard<std::mutex> lock(m_buf);
      empty = sc_loop_icp_buf.empty();
    }
    if (empty) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    std::pair<int, int> loop_idx_pair;
    {
      std::lock_guard<std::mutex> lock(m_buf);
      loop_idx_pair = sc_loop_icp_buf.front();
      sc_loop_icp_buf.pop();
    }
    const int prev_node_idx = loop_idx_pair.first;
    const int curr_node_idx = loop_idx_pair.second;
    gtsam::Pose3 relative_pose = do_icp_virtual_relative(prev_node_idx, curr_node_idx);
    if (!relative_pose.equals(gtsam::Pose3::Identity())) {
      std::lock_guard<std::mutex> lock(mtx_posegraph);
      gt_sam_graph.add(
        gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relative_pose, robust_loop_noise));
    }
  }
}

void OptimizationLOAM::process_isam() {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::microseconds(int(1e6 / std::max(1e-6, graph_update_frequency))));
    if (kill_switch) break;
    if (gt_sam_graph_made) {
      std::lock_guard<std::mutex> lock(mtx_posegraph);
      run_isam2opt();
    }
  }
}

void OptimizationLOAM::process_viz_map() {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::microseconds(int(1e6 / std::max(1e-6, vizmap_frequency))));
    if (kill_switch) break;
    if (recent_idx_updated <= 1) continue;
    laser_cloud_map_pgo->clear();
    {
      std::lock_guard<std::mutex> lock(m_kf);
      for (int node_idx = 0; node_idx < recent_idx_updated; node_idx++) {
        *laser_cloud_map_pgo += *local2global(keyframe_laser_clouds[node_idx], keyframe_poses_updated[node_idx]);
      }
    }
    downsize_filter_map_pgo.setInputCloud(laser_cloud_map_pgo);
    downsize_filter_map_pgo.filter(*laser_cloud_map_pgo);
    map_seq++;
  }
}

bool OptimizationLOAM::get_latest_pose(double& stamp, Pose6D& pose) const {
  std::lock_guard<std::mutex> lock(m_kf);
  if (recent_idx_updated < 0 || recent_idx_updated >= int(keyframe_poses_updated.size())) return false;
  stamp = keyframe_times[recent_idx_updated];
  pose = keyframe_poses_updated[recent_idx_updated];
  return true;
}

}  // namespace optimization_loam
}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::optimization_loam::OptimizationLOAM();
}
