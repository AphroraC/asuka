#include "asuka/odometry/superlio/odometry_estimation.hpp"

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
  M6d htvh = M6d::Zero();
  V6d htvr = V6d::Zero();
  ThreadAcc() : htvh(M6d::Zero()), htvr(V6d::Zero()) {}
};

}  // namespace

bool OdometryEstimation::calc_plane_coeff(int n, const std::array<V3, 5>& points, std::array<double, 4>& abcd) {
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
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}

bool OdometryEstimation::compute_error(
  const std::array<double, 4>& abcd,
  const V3& point,
  float length,
  Scalar& error) {
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config config(GlobalConfig::get_config_path("config_odometry"));
  kf_max_iterations = config.param<int>("kf", "max_iterations", 4);
  kf_align_gravity = config.param<bool>("kf", "align_gravity", true);
  kf_quit_eps = config.param<double>("kf", "quit_eps", 0.001);
  imu_na = config.param<double>("imu", "imu_na", 0.1);
  imu_ng = config.param<double>("imu", "imu_ng", 0.1);
  imu_nba = config.param<double>("imu", "imu_nba", 0.0001);
  imu_nbg = config.param<double>("imu", "imu_nbg", 0.0001);
  gravity_norm = config.param<double>("imu", "gravity_norm", 9.7946);
  ivox_capacity = static_cast<std::size_t>(config.param<int>("hash_map", "hash_capacity", 2000000));
  ivox_resolution = static_cast<float>(config.param<double>("hash_map", "vox_resolution", 0.5));
  voxel_filter_size = static_cast<float>(config.param<double>("preprocess", "voxel_filter_size", 0.5));
  enable_downsample = config.param<bool>("preprocess", "enable_downsample", true);
  time_eva = config.param<bool>("eva", "timer", false);

  lidar_buffer.set_capacity(config.param<int>("odometry", "lidar_buffer_capacity", 50));
  measures.imu.set_capacity(config.param<int>("odometry", "imu_buffer_capacity", 1000));

  const std::vector<double> odom =
    config.param<std::vector<double>>("extrinsic", "odom_robo", std::vector<double>(6, 0.0));
  V3 odom_t(static_cast<Scalar>(odom[0]), static_cast<Scalar>(odom[1]), static_cast<Scalar>(odom[2]));
  Eigen::Matrix3d temp_r = (Eigen::AngleAxisd(odom[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(odom[4] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(odom[3] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();
  M3 odom_r = temp_r.transpose().cast<Scalar>();
  odom_robo = Se3(odom_r, odom_t);
  lidar_robo_yaw =
    Eigen::AngleAxisd(odom[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<Scalar>();

  const Config sensor_config(GlobalConfig::get_config_path("config_sensor"));
  std::string lidar_key;
  if (sensor_config.has("livox")) {
    lidar_key = "livox";
  } else if (sensor_config.has("robosense")) {
    lidar_key = "robosense";
  } else {
    throw std::runtime_error("config_sensor: no active lidar block");
  }
  std::vector<std::string> nest{lidar_key};
  Eigen::Vector3d t_ext =
    sensor_config.param_nested<Eigen::Vector3d>(nest, "extrinsic_T", Eigen::Vector3d(-0.011, -0.02329, 0.04412));
  Eigen::Matrix3d r_ext = sensor_config.param_nested<Eigen::Matrix3d>(nest, "extrinsic_R", Eigen::Matrix3d::Identity());
  M3 r_lidar_imu = r_ext.cast<Scalar>();
  V3 t_lidar_imu = t_ext.cast<Scalar>();
  lidar_imu = Se3(r_lidar_imu, t_lidar_imu);

  const Config ros_config(GlobalConfig::get_config_path("config_ros"));
  save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);

  ivox = std::make_shared<OctVoxMapType>(typename OctVoxMapType::Options(ivox_resolution, ivox_capacity));
  kf = std::make_shared<Eskf>();
  voxel_grid_filter.set_leaf_size(voxel_filter_size);

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
    gravity_norm,
    kf_max_iterations,
    ivox_capacity,
    ivox_resolution,
    voxel_filter_size);
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

void OdometryEstimation::clear_buffers() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  imu_preprocess->imu_buffer.clear();
  imu_preprocess->last_timestamp = -1.0;
  imu_preprocess->reset_accumulators();
  lidar_pushed = false;
  last_timestamp_lidar = -1.0;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return map_cloud;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    imu_preprocess->insert_imu(imu);
  }
  Callbacks::on_insert_imu(imu);
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
  Callbacks::on_insert_frame(stamp, cloud);
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
  if (time_eva) {
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

  V3 mean_gyro = imu_preprocess->mean_gyro;
  V3 mean_acce = imu_preprocess->mean_acc;

  V3 gravity = -mean_acce * gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, -static_cast<Scalar>(gravity_norm));
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 r_yaw_inv = Eigen::AngleAxis<Scalar>(-static_cast<Scalar>(yaw), V3::UnitZ()).toRotationMatrix();
  M3 rot = lidar_robo_yaw * r_yaw_inv * init_rot;

  Eskf::Options options;
  options.gyro_var = imu_ng;
  options.acce_var = imu_na;
  options.bias_gyro_var = imu_nbg;
  options.bias_acce_var = imu_nba;
  options.num_iterations = kf_max_iterations;
  options.quit_eps = kf_quit_eps;

  float imu_scale = static_cast<float>(gravity_norm / mean_acce.norm());
  kf->set_initial_conditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf->get_sys_state();
  state.rot = So3(rot);
  state.p = odom_robo.t;
  state.timestamp = measures.imu.back().secs;
  kf->set_x(state);
  sys_init_pose = kf->get_se3();
  return true;
}

bool OdometryEstimation::map_init() {
  frame_num++;

  std::size_t ptsize = measures.lidar.pc->size();
  points_world.resize(ptsize);

  const Se3 transform = sys_init_pose * lidar_imu;

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
      auto& point_pcl = measures.lidar.pc->points[idx];
      V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
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

  const M3 tli_r = lidar_imu.r;
  const V3 tli_t = lidar_imu.t;
  const Se3 t_end = kf->get_se3();
  const M3 r_inv = t_end.r.transpose();
  const V3 t_end_t = t_end.t;
  const double start_time = measures.lidar.start_time;
  auto& raw_pc = measures.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full->resize(ptsize);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, ptsize), [&](const tbb::blocked_range<size_t>& r) {
    M3 r_h, r_t;
    V3 p_h, v_h, acc_t, w_t;
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
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = r_inv * (tli_r * raw + tli_t);
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
      M3 r_i = Quat(r_h).slerp(s, Quat(r_t)).toRotationMatrix();
      V3 p_i = p_h + v_h * tau + 0.5 * acc_t * tau * tau;
      V3 t_ei = p_i - t_end_t;
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = r_inv * (r_i * (tli_r * raw + tli_t) + t_ei);
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
    points_body[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    lengths[i] = points_body[i].norm();
  }

  ivox->reset_max_group();
  int iter_num = 0;

  kf->update_observe([&, this](const Eskf::KfState& kf_state, M6& htvh, V6& htvr) {
    const Se3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d r_transpose = (pose.r.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadAcc> tls_acc;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, effect_knn_num), [&](const tbb::blocked_range<size_t>& r) {
      KnnHeapType top_k;
      auto& local_acc = tls_acc.local();
      for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
        int idx = effect_knn_idxs[r_s];
        V3& point_body = points_body[idx];
        V3 point_world = pose * point_body;

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
        Scalar error;
        effect_mask[idx] = compute_error(abcd, point_world, lengths[idx], error);
        if (!effect_mask[idx]) continue;

        {
          V3d normvec(abcd[0], abcd[1], abcd[2]);
          V3d nb = r_transpose * normvec;
          V3d point_body_d = point_body.cast<double>();
          V6d j;
          j.head<3>() = point_body_d.cross(nb);
          j.tail<3>() = normvec;

          local_acc.htvh += j * 1000 * j.transpose();
          local_acc.htvr -= j * 1000 * error;
        }
      }
    });

    M6d sum_htvh = M6d::Zero();
    V6d sum_htvr = V6d::Zero();
    for (const auto& local_acc : tls_acc) {
      sum_htvh += local_acc.htvh;
      sum_htvr += local_acc.htvr;
    }
    htvh = sum_htvh.cast<Scalar>();
    htvr = sum_htvr.cast<Scalar>();

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
  frame->stamp = state.timestamp;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = T_world_imu;
  frame->v_world_imu = state.v.cast<double>();
  SysState sys_state = kf->get_sys_state();
  frame->imu_bias.head<3>() = sys_state.bg.cast<double>();
  frame->imu_bias.tail<3>() = sys_state.ba.cast<double>();

  // ds_undistort is a reused member buffer that the update loop also reads:
  // give the frame its own copy so observers may hold it as long as they like.
  frame->cloud_imu = PointCloudT::Ptr(new PointCloudT(*ds_undistort));

  Callbacks::on_new_frame(frame);

  if (save_map_en) {
    *map_cloud += *world_pc;
  }

  logger->debug(
    "[superlio] scan stamp={:.3f} ds={} effect_knn={} map_size={}",
    measures.lidar.start_time,
    ds_undistort->size(),
    effect_knn_num,
    ivox ? 0 : 0);
}

}  // namespace superlio
}  // namespace asuka
