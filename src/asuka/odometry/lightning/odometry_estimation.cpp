#include <asuka/odometry/lightning/odometry_estimation.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <execution>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace lightning {

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  ::lightning::fasterlio::NUM_MAX_ITERATIONS = odometry_config.param<int>("odometry", "max_iteration", 8);
  ::lightning::fasterlio::ESTI_PLANE_THRESHOLD = odometry_config.param<float>("odometry", "esti_plane_threshold", 0.1f);

  use_aa = odometry_config.param<bool>("odometry", "use_aa", false);
  keep_first_imu_estimation = odometry_config.param<bool>("odometry", "keep_first_imu_estimation", false);
  enable_icp_part = odometry_config.param<bool>("odometry", "enable_icp_part", true);
  plane_icp_weight = odometry_config.param<double>("odometry", "plane_icp_weight", 1.0);
  icp_weight = odometry_config.param<double>("odometry", "icp_weight", 100.0);
  min_pts = odometry_config.param<int>("odometry", "min_pts", 300);
  kf_dis_th = odometry_config.param<double>("odometry", "kf_dis_th", 2.0);
  kf_angle_th = odometry_config.param<double>("odometry", "kf_angle_th", 15.0) * M_PI / 180.0;
  skip_lidar_num = odometry_config.param<int>("odometry", "skip_lidar_num", 5);
  enable_skip_lidar = skip_lidar_num > 0;
  proj_kfs_en = odometry_config.param<bool>("odometry", "proj_kfs", false);
  max_proj_kfs = odometry_config.param<int>("odometry", "max_proj_kfs", 5);

  lidar_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  time_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  imu_buffer.set_capacity(odometry_config.param<int>("odometry", "imu_buffer_capacity", 1000));

  const double filter_size_scan = odometry_config.param<double>("mapping", "filter_size_scan", 0.5);
  filter_size_map_min = odometry_config.param<double>("mapping", "filter_size_map", 0.5);
  ivox_options.resolution_ = odometry_config.param<float>("mapping", "ivox_grid_resolution", 0.2);
  const int ivox_nearby_type = odometry_config.param<int>("mapping", "ivox_nearby_type", 18);

  const Config sensor_config(GlobalConfig::get_config_path("config_sensor"));
  std::string lidar_key;
  if (sensor_config.has("livox"))
    lidar_key = "livox";
  else if (sensor_config.has("robosense"))
    lidar_key = "robosense";
  else
    throw std::runtime_error("config_sensor: no active lidar block");
  std::vector<std::string> nest{lidar_key};
  const std::vector<double> extrinT =
    sensor_config.param_nested<std::vector<double>>(nest, "extrinsic_T", {0.0, 0.0, 0.0});
  const std::vector<double> extrinR =
    sensor_config.param_nested<std::vector<double>>(nest, "extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  offset_t_lidar_fixed = ::lightning::math::VecFromArray<double>(extrinT);
  offset_r_lidar_fixed = ::lightning::math::MatFromArray<double>(extrinR);

  voxel_scan.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

  if (ivox_nearby_type == 0) {
    ivox_options.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }

  ivox = std::make_shared<IVoxType>(ivox_options);

  ::lightning::ESKF::Options eskf_options;
  eskf_options.max_iterations_ = ::lightning::fasterlio::NUM_MAX_ITERATIONS;
  eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ::lightning::ESKF::state_dim_, 1>::Ones();
  eskf_options.lidar_obs_func_ = [this](::lightning::NavState& s, ::lightning::ESKF::CustomObservationModel& obs) {
    obs_model(s, obs);
  };
  eskf_options.use_aa_ = use_aa;
  kf.Init(eskf_options);

  (void)keep_first_imu_estimation;

  logger->debug("::lightning::OdometryEstimation initialized");
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

void OdometryEstimation::stop() {
  kill_switch = true;
  std::lock_guard<std::mutex> lock(mtx_buffer);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
  lidar_pushed = false;
}

void OdometryEstimation::clear_buffers() {
  std::lock_guard<std::mutex> lock(mtx_buffer);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
  lidar_pushed = false;
  last_timestamp_imu = -1.0;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  std::lock_guard<std::mutex> lock(mtx_buffer);
  ::lightning::CloudPtr lmap = get_global_map(true);
  PointCloudT::Ptr out(new PointCloudT());
  *out = *lmap;
  return out;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  auto limu = std::make_shared<::lightning::IMU>();
  limu->timestamp = imu->stamp;
  limu->angular_velocity = imu->angular_vel;
  limu->linear_acceleration = imu->linear_acc;

  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (imu->stamp < last_timestamp_imu) {
      logger->warn("imu loop back, clear buffer");
      imu_buffer.clear();
    }

    if (imu_preprocess->is_initialized()) {
      kf_imu.Predict(
        imu->stamp - last_timestamp_imu,
        imu_preprocess->q(),
        limu->angular_velocity,
        limu->linear_acceleration);
    }

    last_timestamp_imu = imu->stamp;
    imu_buffer.push_back(limu);
  }

  Callbacks::on_insert_imu(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (stamp < last_timestamp_lidar) {
      logger->warn("lidar loop back, clear buffer");
      lidar_buffer.clear();
    }
    PointCloudT::Ptr copy(new PointCloudT(*cloud));
    lidar_buffer.push_back(copy);
    time_buffer.push_back(stamp);
    last_timestamp_lidar = stamp;
  }
  Callbacks::on_insert_frame(stamp, cloud);
}

int OdometryEstimation::workload() const {
  return static_cast<int>(lidar_buffer.size());
}

bool OdometryEstimation::process_once() {
  if (kill_switch) return false;
  ImuMeasureGroup measures;
  if (!sync_packages(measures)) {
    return false;
  }
  process_scan(measures);
  return true;
}

bool OdometryEstimation::sync_packages(ImuMeasureGroup& measures) {
  std::lock_guard<std::mutex> lock(mtx_buffer);

  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  if (!lidar_pushed) {
    measures.lidar = lidar_buffer.front();
    measures.lidar_beg_time = time_buffer.front();

    if (measures.lidar->points.size() <= 1) {
      lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime;
      logger->warn("Too few input point cloud!");
    } else if (measures.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
      lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = measures.lidar_beg_time + measures.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime += (measures.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;

      if ((lidar_end_time - measures.lidar_beg_time) > 5 * ::lightning::lo::lidar_time_interval) {
        lidar_end_time = measures.lidar_beg_time + ::lightning::lo::lidar_time_interval;
        lidar_mean_scantime = ::lightning::lo::lidar_time_interval;
      }
    }

    ::lightning::lo::lidar_time_interval = lidar_mean_scantime;
    measures.lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  double imu_time = imu_buffer.front()->timestamp;
  measures.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = imu_buffer.front()->timestamp;
    if (imu_time > lidar_end_time) {
      break;
    }

    auto asukaimu = std::make_shared<ImuData>();
    asukaimu->stamp = imu_buffer.front()->timestamp;
    asukaimu->angular_vel = imu_buffer.front()->angular_velocity;
    asukaimu->linear_acc = imu_buffer.front()->linear_acceleration;
    measures.imu.push_back(asukaimu);

    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;

  return true;
}

void OdometryEstimation::process_scan(ImuMeasureGroup& meas) {
  scan_undistort->clear();
  imu_preprocess->process(meas, kf, scan_undistort);

  if (scan_undistort->empty() || (scan_undistort == nullptr)) {
    logger->warn("No point, skip this scan!");
    return;
  }

  if (first_scan_flag) {
    logger->info("first scan pts: {}", scan_undistort->size());

    state_point = kf.GetX();
    scan_down_world->resize(scan_undistort->size());
    for (int i = 0; i < scan_undistort->size(); i++) {
      point_body_to_world(scan_undistort->points[i], scan_down_world->points[i]);
    }
    ivox->AddPoints(scan_down_world->points);

    first_lidar_time = meas.lidar_beg_time;
    state_point.timestamp_ = lidar_end_time;
    first_scan_flag = false;
    return;
  }

  if (enable_skip_lidar) {
    skip_lidar_cnt++;
    skip_lidar_cnt = skip_lidar_cnt % skip_lidar_num;

    if (skip_lidar_cnt != 0) {
      return;
    }
  }

  if (last_lidar_time > 0 && (meas.lidar_beg_time - last_lidar_time) > 0.5) {
    logger->warn("lidar dropout detected: {:.3f}s", meas.lidar_beg_time - last_lidar_time);
  }
  last_lidar_time = meas.lidar_beg_time;

  ekf_inited_flag = (meas.lidar_beg_time - first_lidar_time) >= ::lightning::fasterlio::INIT_TIME;

  voxel_scan.setInputCloud(scan_undistort);
  voxel_scan.filter(*scan_down_body);

  int cur_pts = scan_down_body->size();

  if (cur_pts < (scan_undistort->size() * 0.1) || cur_pts < min_pts) {
    auto v = voxel_scan;
    v.setLeafSize(0.1, 0.1, 0.1);
    v.setInputCloud(scan_undistort);
    v.filter(*scan_down_body);
    cur_pts = scan_down_body->size();
  }

  if (cur_pts < 5) {
    logger->warn("Too few points, skip this scan! {} {}", scan_undistort->size(), scan_down_body->size());
    return;
  }

  scan_down_world->resize(cur_pts);
  nearest_points.resize(cur_pts);

  residuals.resize(cur_pts, 0);
  point_selected_surf.resize(cur_pts, 1);
  point_selected_icp.resize(cur_pts, 1);
  plane_coef.resize(cur_pts, ::lightning::Vec4f::Zero());

  auto pred_state = kf.GetX();

  kf.Update(::lightning::ESKF::ObsType::LIDAR, 1.0);

  state_point = kf.GetX();
  state_point.timestamp_ = lidar_end_time;

  const double delta_translation = (pred_state.pos_ - state_point.pos_).norm();
  (void)delta_translation;
  const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point.rot_).log().norm() * 180.0 / M_PI;
  (void)delta_rotation_deg;

  logger->debug(
    "[mapping] in {} down {} grids {} eff {}/{}",
    scan_undistort->size(),
    cur_pts,
    ivox->NumValidGrids(),
    effect_feat_surf,
    effect_feat_icp);

  if (last_kf == nullptr) {
    make_kf();
  } else {
    ::lightning::SE3 last_pose = last_kf->GetLIOPose();
    ::lightning::SE3 cur_pose = state_point.GetPose();
    if (
      (last_pose.translation() - cur_pose.translation()).norm() > kf_dis_th ||
      (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > kf_angle_th) {
      make_kf();
    }
  }

  kf_imu = kf;
  if (!meas.imu.empty()) {
    double t = meas.imu.back()->stamp;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    for (auto& imu : imu_buffer) {
      double dt = imu->timestamp - t;
      kf_imu.Predict(dt, imu_preprocess->q(), imu->angular_velocity, imu->linear_acceleration);
      t = imu->timestamp;
    }
  }

  logger->debug(
    "LIO state: {:.3f} {:.3f} {:.3f}, vel: {:.3f} {:.3f} {:.3f}",
    state_point.pos_.x(),
    state_point.pos_.y(),
    state_point.pos_.z(),
    state_point.vel_.x(),
    state_point.vel_.y(),
    state_point.vel_.z());
}

void OdometryEstimation::point_body_to_world(const ::lightning::PointType& pi, ::lightning::PointType& po) {
  ::lightning::Vec3d p_global(
    state_point.rot_ * (offset_r_lidar_fixed * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed) +
    state_point.pos_);

  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

void OdometryEstimation::map_incremental() {
  ::lightning::PointVector points_to_add;
  ::lightning::PointVector point_no_need_downsample;

  size_t cur_pts = scan_down_body->size();
  points_to_add.reserve(cur_pts);
  point_no_need_downsample.reserve(cur_pts);

  std::vector<size_t> index(cur_pts);
  for (size_t i = 0; i < cur_pts; ++i) {
    index[i] = i;
  }

  std::for_each(index.begin(), index.end(), [&](const size_t& i) {
    point_body_to_world(scan_down_body->points[i], scan_down_world->points[i]);

    ::lightning::PointType& point_world = scan_down_world->points[i];
    if (!nearest_points[i].empty() && ekf_inited_flag) {
      const ::lightning::PointVector& points_near = nearest_points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;

      Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

      if (
        fabs(dis_2_center.x()) > 0.5 * filter_size_map_min && fabs(dis_2_center.y()) > 0.5 * filter_size_map_min &&
        fabs(dis_2_center.z()) > 0.5 * filter_size_map_min) {
        point_no_need_downsample.emplace_back(point_world);
        return;
      }

      bool need_add = true;
      float dist = ::lightning::math::calc_dist(point_world.getVector3fMap(), center);
      if (points_near.size() >= ::lightning::fasterlio::NUM_MATCH_POINTS) {
        for (int readd_i = 0; readd_i < ::lightning::fasterlio::NUM_MATCH_POINTS; readd_i++) {
          if (::lightning::math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
            need_add = false;
            break;
          }
        }
      }

      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  });

  ivox->AddPoints(points_to_add);
  ivox->AddPoints(point_no_need_downsample);
}

void OdometryEstimation::obs_model(::lightning::NavState& s, ::lightning::ESKF::CustomObservationModel& obs) {
  int cnt_pts = scan_down_body->size();

  std::vector<size_t> index(cnt_pts);
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  ::lightning::Mat3f R_wl = (s.rot_.matrix() * offset_r_lidar_fixed).cast<float>();
  ::lightning::Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed + s.pos_).cast<float>();

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
    ::lightning::PointType& point_body = scan_down_body->points[i];
    ::lightning::PointType& point_world = scan_down_world->points[i];

    ::lightning::Vec3f p_body = point_body.getVector3fMap();
    point_world.getVector3fMap() = R_wl * p_body + t_wl;
    point_world.intensity = point_body.intensity;

    auto& points_near = nearest_points[i];
    points_near.clear();

    ivox->GetClosestPoint(point_world, points_near, ::lightning::fasterlio::NUM_MATCH_POINTS);
    point_selected_surf[i] = points_near.size() >= ::lightning::fasterlio::MIN_NUM_MATCH_POINTS;

    point_selected_icp[i] = point_selected_surf[i];

    if (point_selected_surf[i]) {
      point_selected_surf[i] =
        ::lightning::math::esti_plane(plane_coef[i], points_near, ::lightning::fasterlio::ESTI_PLANE_THRESHOLD);
    }

    if (point_selected_surf[i]) {
      auto temp = point_world.getVector4fMap();
      temp[3] = 1.0;
      float pd2 = plane_coef[i].dot(temp);

      if (p_body.norm() > 81 * pd2 * pd2) {
        point_selected_surf[i] = true;
        residuals[i] = pd2;
      } else {
        point_selected_surf[i] = false;
      }
    }
  });

  effect_feat_surf = 0;
  effect_feat_icp = 0;

  corr_pts.resize(cnt_pts);
  corr_norm.resize(cnt_pts);
  for (int i = 0; i < cnt_pts; i++) {
    if (point_selected_surf[i]) {
      corr_norm[effect_feat_surf] = plane_coef[i];
      corr_pts[effect_feat_surf] = scan_down_body->points[i].getVector4fMap();
      corr_pts[effect_feat_surf][3] = residuals[i];

      effect_feat_surf++;
    }

    if (point_selected_icp[i]) {
      effect_feat_icp++;
    }
  }

  corr_pts.resize(effect_feat_surf);
  corr_norm.resize(effect_feat_surf);

  if (effect_feat_surf < 20) {
    obs.valid_ = false;
    logger->warn("No enough effective surface points: {}, icp: {}", effect_feat_surf, effect_feat_icp);
    return;
  }

  index.resize(effect_feat_surf);
  const ::lightning::Mat3f off_R = offset_r_lidar_fixed.cast<float>();
  const ::lightning::Vec3f off_t = offset_t_lidar_fixed.cast<float>();
  const ::lightning::Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

  obs.HTH_.setZero();
  obs.HTr_.setZero();

  std::vector<::lightning::Mat6d> JTJ(effect_feat_surf);
  std::vector<::lightning::Vec6d> JTr(effect_feat_surf);

  std::vector<double> res_sq(index.size());

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
    ::lightning::Vec3f point_this_be = corr_pts[i].head<3>();
    ::lightning::Vec3f point_this = off_R * point_this_be + off_t;
    ::lightning::Mat3f point_crossmat = ::lightning::math::SKEW_SYM_MATRIX(point_this);

    ::lightning::Vec3f norm_vec = corr_norm[i].head<3>();

    ::lightning::Vec3f C(Rt * norm_vec);
    ::lightning::Vec3f A(point_crossmat * C);

    Eigen::Matrix<double, 1, ::lightning::ESKF::pose_obs_dim_> J;
    J.setZero();
    J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

    float res = -corr_pts[i][3];

    double w = 1.0;

    JTJ[i] = (J.transpose() * J).eval() * w;
    JTr[i] = J.transpose() * res * w;

    res_sq[i] = res * res;
  });

  for (int i = 0; i < static_cast<int>(index.size()); ++i) {
    obs.HTH_ += JTJ[i] * plane_icp_weight;
    obs.HTr_ += JTr[i] * plane_icp_weight;
  }

  if (!res_sq.empty()) {
    std::sort(res_sq.begin(), res_sq.end());
    obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
    obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
  }

  if (enable_icp_part) {
    JTJ.resize(cnt_pts);
    JTr.resize(cnt_pts);

    std::vector<size_t> icp_index(cnt_pts);
    for (size_t i = 0; i < icp_index.size(); ++i) {
      icp_index[i] = i;
    }

    std::for_each(std::execution::par_unseq, icp_index.begin(), icp_index.end(), [&](const size_t& i) {
      if (point_selected_icp[i] == false) {
        return;
      }

      ::lightning::Vec3d q = scan_down_body->points[i].getVector3fMap().cast<double>();
      ::lightning::Vec3d qs = scan_down_world->points[i].getVector3fMap().cast<double>();

      Eigen::Matrix<double, 3, ::lightning::ESKF::pose_obs_dim_> J;
      J.setZero();

      J.block<3, 3>(0, 0) = ::lightning::Mat3d::Identity();

      J.block<3, 3>(0, 3) = -(s.rot_.matrix() * offset_r_lidar_fixed) * ::lightning::SO3::hat(q);

      ::lightning::Vec3d e = qs - nearest_points[i][0].getVector3fMap().cast<double>();

      if (e.norm() > 0.5) {
        point_selected_icp[i] = false;
        return;
      }

      JTJ[i] = J.transpose() * J;
      JTr[i] = -J.transpose() * e;
    });

    for (int i = 0; i < cnt_pts; ++i) {
      if (point_selected_icp[i] == false) {
        continue;
      }
      obs.HTH_ += JTJ[i] * icp_weight;
      obs.HTr_ += JTr[i] * icp_weight;
    }
  }
}

void OdometryEstimation::make_kf() {
  ::lightning::Keyframe::Ptr kf = std::make_shared<::lightning::Keyframe>(kf_id++, scan_undistort, state_point);

  if (last_kf) {
    ::lightning::SE3 delta = last_kf->GetLIOPose().inverse() * kf->GetLIOPose();
    kf->SetOptPose(last_kf->GetOptPose() * delta);
  } else {
    kf->SetOptPose(kf->GetLIOPose());
  }

  kf->SetState(state_point);

  all_keyframes.emplace_back(kf);
  last_kf = kf;

  map_incremental();

  if (proj_kfs_en) {
    if (static_cast<int>(proj_kfs.size()) >= max_proj_kfs) {
      auto last = proj_kfs.back();
      ::lightning::SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();
      if (!(delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI)) {
        proj_kfs.pop_front();
        proj_kfs.emplace_back(kf);
      }
    } else {
      proj_kfs.emplace_back(kf);
    }
  }

  Eigen::Isometry3d T_world_imu(state_point.rot_.matrix());
  T_world_imu.pretranslate(state_point.pos_);

  auto frame = std::make_shared<asuka::KeyFrame>();
  frame->id = frame_counter++;
  frame->stamp = state_point.timestamp_;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = T_world_imu;
  frame->v_world_imu = state_point.vel_;
  frame->imu_bias.head<3>() = state_point.bg_;
  frame->imu_bias.tail<3>().setZero();

  // scan_undistort is in the lidar frame: apply the fixed extrinsic so the
  // KeyFrame cloud matches its IMU-frame semantics (same convention as the
  // observation model).
  PointCloudT::Ptr cloud_imu(new PointCloudT());
  cloud_imu->resize(scan_undistort->size());
  for (std::size_t i = 0; i < scan_undistort->size(); ++i) {
    const auto& pt = scan_undistort->points[i];
    ::lightning::Vec3d p_imu = offset_r_lidar_fixed * ::lightning::Vec3d(pt.x, pt.y, pt.z) + offset_t_lidar_fixed;
    cloud_imu->points[i].x = static_cast<float>(p_imu(0));
    cloud_imu->points[i].y = static_cast<float>(p_imu(1));
    cloud_imu->points[i].z = static_cast<float>(p_imu(2));
    cloud_imu->points[i].intensity = pt.intensity;
  }
  frame->cloud_imu = cloud_imu;

  Callbacks::on_new_frame(frame);
}

::lightning::CloudPtr OdometryEstimation::get_global_map(bool use_lio_pose, bool use_voxel, float res) {
  ::lightning::CloudPtr global_map(new ::lightning::PointCloudType);

  pcl::VoxelGrid<::lightning::PointType> voxel;
  voxel.setLeafSize(res, res, res);

  for (auto& kf : all_keyframes) {
    ::lightning::CloudPtr cloud = kf->GetCloud();

    ::lightning::CloudPtr cloud_filter(new ::lightning::PointCloudType);

    if (use_voxel) {
      voxel.setInputCloud(cloud);
      voxel.filter(*cloud_filter);
    } else {
      cloud_filter = cloud;
    }

    ::lightning::CloudPtr cloud_trans(new ::lightning::PointCloudType);

    // Keyframe clouds are lidar frame while the poses are IMU frame: apply
    // the fixed extrinsic first (upstream lightning-lm mixes the two).
    Eigen::Matrix4d t_li = Eigen::Matrix4d::Identity();
    t_li.block<3, 3>(0, 0) = offset_r_lidar_fixed;
    t_li.block<3, 1>(0, 3) = offset_t_lidar_fixed;
    if (use_lio_pose) {
      pcl::transformPointCloud(*cloud_filter, *cloud_trans, (kf->GetLIOPose().matrix() * t_li).eval());
    } else {
      pcl::transformPointCloud(*cloud_filter, *cloud_trans, (kf->GetOptPose().matrix() * t_li).eval());
    }

    *global_map += *cloud_trans;
  }

  ::lightning::CloudPtr global_map_filtered(new ::lightning::PointCloudType);
  if (use_voxel) {
    voxel.setInputCloud(global_map);
    voxel.filter(*global_map_filtered);
  } else {
    global_map_filtered = global_map;
  }

  global_map_filtered->is_dense = false;
  global_map_filtered->height = 1;
  global_map_filtered->width = global_map_filtered->size();

  return global_map_filtered;
}

}  // namespace lightning
}  // namespace asuka
