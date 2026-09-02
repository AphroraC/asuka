#include <algorithm>
#include <cmath>
#include <cstring>

#include <omp.h>

#include <asuka/odometry/batchlio/odometry_estimation.hpp>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace batchlio {
namespace {

void reset_state_input(batch::state_input& s) {
  s.pos = batch::vect3(batch::vect3::Zero());
  s.rot = batch::SO3(Eigen::Matrix3d::Identity());
  s.offset_R_L_I = batch::SO3(Eigen::Matrix3d::Identity());
  s.offset_T_L_I = batch::vect3(batch::vect3::Zero());
  s.vel = batch::vect3(batch::vect3::Zero());
  s.bg = batch::vect3(batch::vect3::Zero());
  s.ba = batch::vect3(batch::vect3::Zero());
  s.gravity = batch::vect3(batch::vect3::Zero());
}

void reset_state_output(batch::state_output& s) {
  s.pos = batch::vect3(batch::vect3::Zero());
  s.rot = batch::SO3(Eigen::Matrix3d::Identity());
  s.offset_R_L_I = batch::SO3(Eigen::Matrix3d::Identity());
  s.offset_T_L_I = batch::vect3(batch::vect3::Zero());
  s.vel = batch::vect3(batch::vect3::Zero());
  s.omg = batch::vect3(batch::vect3::Zero());
  s.acc = batch::vect3(batch::vect3::Zero());
  s.gravity = batch::vect3(batch::vect3::Zero());
  s.bg = batch::vect3(batch::vect3::Zero());
  s.ba = batch::vect3(batch::vect3::Zero());
}

}  // namespace

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));

  batch_dt = odometry_config.param<double>("batch", "dt", 0.001);
  batch_deskew = odometry_config.param<bool>("batch", "deskew", true);
  batch_omp = odometry_config.param<bool>("batch", "enable_openmp", false);

  imu_enabled = odometry_config.param<bool>("mapping", "imu_en", true);
  lidar_time_inte = odometry_config.param<double>("mapping", "lidar_time_inte", 0.1);
  satu_acc = odometry_config.param<double>("mapping", "satu_acc", 3.0);
  satu_gyro = odometry_config.param<double>("mapping", "satu_gyro", 35.0);
  acc_norm = odometry_config.param<double>("mapping", "acc_norm", 1.0);
  acc_cov_output = odometry_config.param<double>("mapping", "acc_cov_output", 500.0);
  gyr_cov_output = odometry_config.param<double>("mapping", "gyr_cov_output", 1000.0);
  b_acc_cov = odometry_config.param<double>("mapping", "b_acc_cov", 0.0001);
  b_gyr_cov = odometry_config.param<double>("mapping", "b_gyr_cov", 0.0001);
  imu_meas_acc_cov = odometry_config.param<double>("mapping", "imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov = odometry_config.param<double>("mapping", "imu_meas_omg_cov", 0.1);
  gyr_cov_input = odometry_config.param<double>("mapping", "gyr_cov_input", 0.01);
  acc_cov_input = odometry_config.param<double>("mapping", "acc_cov_input", 0.1);
  vel_cov = odometry_config.param<double>("mapping", "vel_cov", 20.0);
  plane_thr = static_cast<float>(odometry_config.param<double>("mapping", "plane_thr", 0.1));
  match_s = odometry_config.param<double>("mapping", "match_s", 81.0);
  init_map_size = odometry_config.param<int>("mapping", "init_map_size", 100);
  space_down_sample = odometry_config.param<bool>("mapping", "space_down_sample", true);
  use_imu_as_input = odometry_config.param<bool>("mapping", "use_imu_as_input", false);
  prop_at_freq_of_imu = odometry_config.param<bool>("mapping", "prop_at_freq_of_imu", true);
  check_satu = odometry_config.param<bool>("mapping", "check_satu", true);
  gravity_vec = odometry_config.param<Eigen::Vector3d>("mapping", "gravity", Eigen::Vector3d(0.0, 0.0, -9.81));
  gravity_init_vec = odometry_config.param<Eigen::Vector3d>("mapping", "gravity_init", gravity_vec);
  laser_point_cov = odometry_config.param<double>("mapping", "lidar_meas_cov", 0.01);

  lidar_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  time_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  imu_buffer.set_capacity(odometry_config.param<int>("odometry", "imu_buffer_capacity", 1000));
  imu_deque.set_capacity(odometry_config.param<int>("odometry", "imu_buffer_capacity", 1000));

  filter_size_surf_min = odometry_config.param<double>("filter", "filter_size_surf", 0.5);
  filter_size_map_min = odometry_config.param<double>("filter", "filter_size_map", 0.5);

  const Config ros_config(GlobalConfig::get_config_path("config_ros"));
  pcd_save_en = ros_config.param<bool>("ros", "enable_map_saving", false);

  Config sensor_config(GlobalConfig::get_config_path("config_sensor"));
  std::string lidar_key;
  if (sensor_config.has("livox"))
    lidar_key = "livox";
  else if (sensor_config.has("robosense"))
    lidar_key = "robosense";
  else
    throw std::runtime_error("config_sensor: no active lidar block");
  std::vector<std::string> nest{lidar_key};
  extrinsic_T = sensor_config.param_nested<Eigen::Vector3d>(nest, "extrinsic_T", Eigen::Vector3d::Zero());
  extrinsic_R = sensor_config.param_nested<Eigen::Matrix3d>(nest, "extrinsic_R", Eigen::Matrix3d::Identity());

#ifdef _OPENMP
  omp_threads = batch_omp ? std::min(omp_get_max_threads(), 16) : 1;
#endif

  const double resolution = odometry_config.param<double>("mapping", "ivox_grid_resolution", 2.0);
  ivox_options.resolution_ = static_cast<float>(resolution);
  ivox_options.inv_resolution_ = static_cast<float>(1.0 / resolution);
  const int nearby = odometry_config.param<int>("mapping", "ivox_nearby_type", 18);
  if (nearby == 0) {
    ivox_options.nearby_type_ = batch::IVoxType::NearbyType::CENTER;
  } else if (nearby == 6) {
    ivox_options.nearby_type_ = batch::IVoxType::NearbyType::NEARBY6;
  } else if (nearby == 18) {
    ivox_options.nearby_type_ = batch::IVoxType::NearbyType::NEARBY18;
  } else if (nearby == 26) {
    ivox_options.nearby_type_ = batch::IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options.nearby_type_ = batch::IVoxType::NearbyType::NEARBY18;
  }
  ivox = std::make_shared<batch::IVoxType>(ivox_options);
  point_selected_surf.fill(true);

  kf_input.init_dyn_share_modified_2h(
    batch::get_f_input,
    batch::df_dx_input,
    &OdometryEstimation::h_model_input_trampoline);
  kf_output.init_dyn_share_modified_3h(
    batch::get_f_output,
    batch::df_dx_output,
    &OdometryEstimation::h_model_output_trampoline,
    &OdometryEstimation::h_model_imu_output_trampoline);
  // if (extrinsic_est_en) {
  //   kf_input.x_.offset_R_L_I = extrinsic_R;
  //   kf_input.x_.offset_T_L_I = extrinsic_T;
  //   kf_output.x_.offset_R_L_I = extrinsic_R;
  //   kf_output.x_.offset_T_L_I = extrinsic_T;
  // }
  batch::reset_cov(p_input);
  kf_input.change_P(p_input);
  batch::reset_cov_output(p_output);
  kf_output.change_P(p_output);
  q_input = process_noise_cov_input();
  q_output = process_noise_cov_output();

  downsize_filter_surf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  imu_preprocess->imu_enabled = imu_enabled;
  imu_preprocess->gravity_value = gravity_vec;

  logger->info(
    "batchlio::OdometryEstimation initialized (use_imu_as_input={}, batch_dt={:.4f} s, imu_en={})",
    use_imu_as_input,
    batch_dt,
    imu_enabled);
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

void OdometryEstimation::stop() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
}

void OdometryEstimation::clear_buffers() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
  imu_initialization.clear();
  last_synchronized_imu.reset();
  last_imu_stamp = -1.0;
  last_lidar_stamp = -1.0;
  initializing_imu = true;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  return map_cloud;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (last_imu_stamp > 0.0 && imu->stamp < last_imu_stamp) {
      logger->warn("imu loop back, dropping sample {:.6f} (last {:.6f})", imu->stamp, last_imu_stamp);
      return;
    }
    imu_buffer.push_back(imu);
    last_imu_stamp = imu->stamp;
  }
  Callbacks::on_insert_imu(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) return;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (last_lidar_stamp > 0.0 && stamp < last_lidar_stamp) {
      logger->warn("lidar loop back, dropping frame {:.6f} (last {:.6f})", stamp, last_lidar_stamp);
      return;
    }
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(stamp);
    last_lidar_stamp = stamp;
  }
  Callbacks::on_insert_frame(stamp, cloud);
}

int OdometryEstimation::workload() const {
  return static_cast<int>(lidar_buffer.size());
}

bool OdometryEstimation::process_once() {
  batch::MeasureGroup measurements;
  if (!synchronize(measurements)) return false;
  active = this;
  process(measurements);
  if (initializing_imu && imu_preprocess && !imu_preprocess->imu_need_init) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    initializing_imu = false;
    imu_initialization.clear();
  }
  return true;
}

bool OdometryEstimation::synchronize(batch::MeasureGroup& measurements) {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  if (lidar_buffer.empty()) return false;
  if (imu_enabled && imu_buffer.empty()) return false;
  const auto& cloud = lidar_buffer.front();
  const double begin_time = time_buffer.front();
  double end_time = begin_time;
  if (!cloud->empty()) {
    double max_curvature = 0.0;
    for (const auto& point : *cloud) max_curvature = std::max(max_curvature, static_cast<double>(point.curvature));
    end_time = begin_time + max_curvature * 1e-3;
  }
  if (imu_enabled && last_imu_stamp < end_time) return false;

  measurements.lidar_begin_time = begin_time;
  measurements.lidar_end_time = end_time;
  measurements.lidar_last_time = end_time;
  measurements.lidar = cloud;
  measurements.imu.clear();
  if (imu_enabled) {
    while (!imu_buffer.empty() && imu_buffer.front()->stamp < end_time) {
      measurements.imu.push_back(imu_buffer.front());
      last_synchronized_imu = imu_buffer.front();
      if (initializing_imu) imu_initialization.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    if (!initializing_imu && measurements.imu.empty() && last_synchronized_imu) {
      measurements.imu.push_back(last_synchronized_imu);
    }
    if (initializing_imu) measurements.imu_initialization = imu_initialization;
    if (!imu_buffer.empty()) measurements.imu_after_end = imu_buffer.front();
  }
  lidar_buffer.pop_front();
  time_buffer.pop_front();
  return true;
}

void OdometryEstimation::reset() {
  reset_flag = true;
  imu_deque.clear();
  imu_last = ImuData();
  imu_next = ImuData();
  init_map = false;
  first_scan_flag = true;
  is_first_frame = true;
  time_current = 0.0;
  time_update_last = 0.0;
  time_predict_last_const = 0.0;
  t_last = 0.0;
  first_imu_time = 0.0;
  last_queued_imu_stamp = -1.0;
  map_cloud.reset(new PointCloudT);
  feats_undistort.reset(new PointCloudT);
  init_feats_world.reset(new PointCloudT);
  feats_down_body.reset(new PointCloudT(10000, 1));
  feats_down_world.reset(new PointCloudT(10000, 1));
  normvec.reset(new PointCloudT(100000, 1));
  time_seq.clear();
  pbody_list.clear();
  nearest_points.clear();
  crossmat_list.clear();
  feats_down_size = 0;
  idx = -1;
  k_window = 0;

  reset_state_input(kf_input.x_);
  reset_state_output(kf_output.x_);
  // if (extrinsic_est_en) {
  //   kf_input.x_.offset_R_L_I = extrinsic_R;
  //   kf_input.x_.offset_T_L_I = extrinsic_T;
  //   kf_output.x_.offset_R_L_I = extrinsic_R;
  //   kf_output.x_.offset_T_L_I = extrinsic_T;
  // }
  kf_input.change_P(p_input);
  kf_output.change_P(p_output);
  ivox = std::make_shared<batch::IVoxType>(ivox_options);
  point_selected_surf.fill(true);

  imu_preprocess->reset();
  imu_preprocess->imu_enabled = imu_enabled;
  imu_preprocess->gravity_value = gravity_vec;
  reset_flag = false;
}

void OdometryEstimation::handle_first_scan(const batch::MeasureGroup& measurements) {
  if (!first_scan_flag) return;
  first_lidar_time = measurements.lidar_begin_time;
  first_scan_flag = false;
  if (first_imu_time < 1) {
    if (!measurements.imu.empty()) first_imu_time = measurements.imu.front()->stamp;
  }
  time_current = 0.0;
  if (imu_enabled) {
    kf_input.x_.gravity << gravity_vec(0), gravity_vec(1), gravity_vec(2);
    kf_output.x_.gravity << gravity_vec(0), gravity_vec(1), gravity_vec(2);
    while (!imu_deque.empty() && measurements.lidar_begin_time > imu_next.stamp) {
      imu_deque.pop_front();
      if (imu_deque.empty()) break;
      imu_last = imu_next;
      imu_next = *(imu_deque.front());
    }
  } else {
    kf_input.x_.gravity << gravity_vec(0), gravity_vec(1), gravity_vec(2);
    kf_output.x_.gravity << gravity_vec(0), gravity_vec(1), gravity_vec(2);
    kf_output.x_.acc << gravity_vec(0), gravity_vec(1), gravity_vec(2);
    kf_output.x_.acc *= -1;
    imu_preprocess->imu_need_init = false;
  }
  g_m_s2 =
    std::sqrt(gravity_vec(0) * gravity_vec(0) + gravity_vec(1) * gravity_vec(1) + gravity_vec(2) * gravity_vec(2));
}

void OdometryEstimation::downsample_and_window(const batch::MeasureGroup& measurements) {
  if (space_down_sample) {
    downsize_filter_surf.setInputCloud(feats_undistort);
    downsize_filter_surf.filter(*feats_down_body);
    std::sort(feats_down_body->points.begin(), feats_down_body->points.end(), batch::time_list);
  } else {
    *feats_down_body = *measurements.lidar;
    std::sort(feats_down_body->points.begin(), feats_down_body->points.end(), batch::time_list);
  }
  time_seq = batch::time_compressing_batch(feats_down_body, batch_dt * 1000.0);
  feats_down_size = static_cast<int>(feats_down_body->points.size());
}

void OdometryEstimation::initialize_map(const batch::MeasureGroup& measurements) {
  feats_down_world->resize(feats_undistort->size());
  for (int i = 0; i < static_cast<int>(feats_undistort->size()); i++) {
    point_body_to_world(feats_undistort->points[i], feats_down_world->points[i]);
  }
  for (std::size_t i = 0; i < feats_down_world->size(); i++) {
    init_feats_world->points.emplace_back(feats_down_world->points[i]);
  }
  if (init_feats_world->size() < static_cast<std::size_t>(init_map_size)) {
    init_map = false;
  } else {
    ivox->AddPoints(init_feats_world->points);
    init_feats_world.reset(new PointCloudT);
    init_map = true;
  }
}

void OdometryEstimation::run_output_loop(const batch::MeasureGroup& measurements) {
  bool imu_upda_cov = false;
  effct_feat_num = 0;
  if (time_seq.size() > 0) {
    const double pcl_beg_time = measurements.lidar_begin_time;
    idx = -1;
    for (k_window = 0; k_window < static_cast<int>(time_seq.size()); k_window++) {
      PointT& point_body = feats_down_body->points[idx + time_seq[k_window]];

      time_current = point_body.curvature / 1000.0 + pcl_beg_time;

      if (is_first_frame) {
        if (imu_enabled) {
          while (time_current > imu_next.stamp) {
            imu_deque.pop_front();
            if (imu_deque.empty()) break;
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
          }
          angvel_avr << imu_last.angular_vel(0), imu_last.angular_vel(1), imu_last.angular_vel(2);
          acc_avr << imu_last.linear_acc(0), imu_last.linear_acc(1), imu_last.linear_acc(2);
        }
        is_first_frame = false;
        imu_upda_cov = true;
        time_update_last = time_current;
        time_predict_last_const = time_current;
      }
      if (imu_enabled && !imu_deque.empty()) {
        const bool last_imu = imu_next.stamp == imu_deque.front()->stamp;
        while (imu_next.stamp < time_predict_last_const && !imu_deque.empty()) {
          if (!last_imu) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            break;
          }
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        }
        bool imu_comes = time_current > imu_next.stamp;
        while (imu_comes) {
          imu_upda_cov = true;
          angvel_avr << imu_next.angular_vel(0), imu_next.angular_vel(1), imu_next.angular_vel(2);
          acc_avr << imu_next.linear_acc(0), imu_next.linear_acc(1), imu_next.linear_acc(2);

          double dt = imu_next.stamp - time_predict_last_const;
          kf_output.predict(dt, q_output, input_in, true, false);
          time_predict_last_const = imu_next.stamp;

          {
            double dt_cov = imu_next.stamp - time_update_last;
            if (dt_cov > 0.0) {
              time_update_last = imu_next.stamp;
              kf_output.predict(dt_cov, q_output, input_in, false, true);
              kf_output.update_iterated_dyn_share_IMU();
            }
          }
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
          imu_comes = time_current > imu_next.stamp;
        }
      }
      if (reset_flag) break;

      double dt = time_current - time_predict_last_const;
      if (!prop_at_freq_of_imu) {
        double dt_cov = time_current - time_update_last;
        if (dt_cov > 0.0) {
          kf_output.predict(dt_cov, q_output, input_in, false, true);
          time_update_last = time_current;
        }
      }
      kf_output.predict(dt, q_output, input_in, true, false);
      time_predict_last_const = time_current;

      if (batch_dt > 0.0 && batch_deskew && time_seq[k_window] > 1) {
        const double t_last_ms = feats_down_body->points[idx + time_seq[k_window]].curvature;
        const Eigen::Vector3d omg_b = kf_output.x_.omg;
        const Eigen::Vector3d vel_w = kf_output.x_.vel;
        const Eigen::Matrix3d r_i = kf_output.x_.rot;
        // Deskew operates on IMU-frame points, but feats_down_body is lidar
        // frame: apply the fixed extrinsic first and map the deskewed point
        // back (the downstream observation model applies the extrinsic
        // again). Upstream Batch-LIO skips this and mixes the frames.
        const Eigen::Matrix3d r_li = extrinsic_R;
        const Eigen::Vector3d t_li = extrinsic_T;
        for (int jj = 1; jj <= time_seq[k_window]; jj++) {
          PointT& pb = feats_down_body->points[idx + jj];
          const double dt_j = (pb.curvature - t_last_ms) / 1000.0;
          const Eigen::Vector3d p_imu = r_li * Eigen::Vector3d(pb.x, pb.y, pb.z) + t_li;
          const Eigen::Vector3d pd_imu = batch::deskew_point(p_imu, dt_j, omg_b, vel_w, r_i);
          const Eigen::Vector3d pd = r_li.transpose() * (pd_imu - t_li);
          pb.x = pd(0);
          pb.y = pd(1);
          pb.z = pd(2);
          pbody_list[idx + jj] = pd;
          Eigen::Matrix3d pc;
          pc << SKEW_SYM_MATRX(pd);
          crossmat_list[idx + jj] = pc;
        }
      }

      if (feats_down_size < 1) {
        logger->warn("No point, skip this scan!");
        idx += time_seq[k_window];
        continue;
      }
      if (!kf_output.update_iterated_dyn_share_modified()) {
        idx += time_seq[k_window];
        continue;
      }

      for (int j = 0; j < time_seq[k_window]; j++) {
        PointT& point_body_j = feats_down_body->points[idx + j + 1];
        PointT& point_world_j = feats_down_world->points[idx + j + 1];
        point_body_to_world(point_body_j, point_world_j);
      }

      idx += time_seq[k_window];
    }
  } else {
    if (!imu_deque.empty()) {
      imu_last = imu_next;
      imu_next = *(imu_deque.front());
      while (imu_next.stamp > time_current && (imu_next.stamp < measurements.lidar_begin_time + lidar_time_inte)) {
        if (is_first_frame) {
          {
            {
              while (imu_next.stamp < measurements.lidar_begin_time + lidar_time_inte) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
            break;
          }
          angvel_avr << imu_last.angular_vel(0), imu_last.angular_vel(1), imu_last.angular_vel(2);
          acc_avr << imu_last.linear_acc(0), imu_last.linear_acc(1), imu_last.linear_acc(2);

          imu_upda_cov = true;
          time_update_last = time_current;
          time_predict_last_const = time_current;

          is_first_frame = false;
        }
        time_current = imu_next.stamp;

        if (!is_first_frame) {
          double dt = time_current - time_predict_last_const;
          {
            double dt_cov = time_current - time_update_last;
            if (dt_cov > 0.0) {
              kf_output.predict(dt_cov, q_output, input_in, false, true);
              time_update_last = time_current;
            }
            kf_output.predict(dt, q_output, input_in, true, false);
          }
          time_predict_last_const = time_current;

          angvel_avr << imu_next.angular_vel(0), imu_next.angular_vel(1), imu_next.angular_vel(2);
          acc_avr << imu_next.linear_acc(0), imu_next.linear_acc(1), imu_next.linear_acc(2);
          kf_output.update_iterated_dyn_share_IMU();
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        } else {
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        }
      }
    }
  }
  (void)imu_upda_cov;
}

void OdometryEstimation::run_input_loop(const batch::MeasureGroup& measurements) {
  bool imu_prop_cov = false;
  effct_feat_num = 0;
  if (time_seq.size() > 0) {
    const double pcl_beg_time = measurements.lidar_begin_time;
    idx = -1;
    for (k_window = 0; k_window < static_cast<int>(time_seq.size()); k_window++) {
      PointT& point_body = feats_down_body->points[idx + time_seq[k_window]];
      time_current = point_body.curvature / 1000.0 + pcl_beg_time;
      if (is_first_frame) {
        while (time_current > imu_next.stamp) {
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        }
        imu_prop_cov = true;
        is_first_frame = false;
        t_last = time_current;
        time_update_last = time_current;
        {
          input_in.gyro << imu_last.angular_vel(0), imu_last.angular_vel(1), imu_last.angular_vel(2);
          input_in.acc << imu_last.linear_acc(0), imu_last.linear_acc(1), imu_last.linear_acc(2);
          input_in.acc = input_in.acc * g_m_s2 / acc_norm;
        }
      }
      while (time_current > imu_next.stamp) {
        imu_deque.pop_front();
        input_in.gyro << imu_last.angular_vel(0), imu_last.angular_vel(1), imu_last.angular_vel(2);
        input_in.acc << imu_last.linear_acc(0), imu_last.linear_acc(1), imu_last.linear_acc(2);
        input_in.acc = input_in.acc * g_m_s2 / acc_norm;
        double dt = imu_last.stamp - t_last;
        double dt_cov = imu_last.stamp - time_update_last;
        if (dt_cov > 0.0) {
          kf_input.predict(dt_cov, q_input, input_in, false, true);
          time_update_last = imu_last.stamp;
        }
        kf_input.predict(dt, q_input, input_in, true, false);
        t_last = imu_last.stamp;
        imu_prop_cov = true;
        if (imu_deque.empty()) break;
        imu_last = imu_next;
        imu_next = *(imu_deque.front());
      }
      if (reset_flag) break;

      double dt = time_current - t_last;
      t_last = time_current;
      if (!prop_at_freq_of_imu) {
        double dt_cov = time_current - time_update_last;
        if (dt_cov > 0.0) {
          kf_input.predict(dt_cov, q_input, input_in, false, true);
          time_update_last = time_current;
        }
      }
      kf_input.predict(dt, q_input, input_in, true, false);

      if (feats_down_size < 1) {
        logger->warn("No point, skip this scan!");
        idx += time_seq[k_window];
        continue;
      }
      if (!kf_input.update_iterated_dyn_share_modified()) {
        idx += time_seq[k_window];
        continue;
      }

      for (int j = 0; j < time_seq[k_window]; j++) {
        PointT& point_body_j = feats_down_body->points[idx + j + 1];
        PointT& point_world_j = feats_down_world->points[idx + j + 1];
        point_body_to_world(point_body_j, point_world_j);
      }

      idx += time_seq[k_window];
    }
  } else {
    if (!imu_deque.empty()) {
      imu_last = imu_next;
      imu_next = *(imu_deque.front());
      while (imu_next.stamp > time_current && (imu_next.stamp < measurements.lidar_begin_time + lidar_time_inte)) {
        if (is_first_frame) {
          {
            {
              while (imu_next.stamp < measurements.lidar_begin_time + lidar_time_inte) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
            break;
          }
          imu_prop_cov = true;
          t_last = time_current;
          time_update_last = time_current;
          input_in.gyro << imu_last.angular_vel(0), imu_last.angular_vel(1), imu_last.angular_vel(2);
          input_in.acc << imu_last.linear_acc(0), imu_last.linear_acc(1), imu_last.linear_acc(2);
          input_in.acc = input_in.acc * g_m_s2 / acc_norm;
          is_first_frame = false;
        }
        time_current = imu_next.stamp;
        if (!is_first_frame) {
          double dt_cov = time_current - time_update_last;
          if (dt_cov > 0.0) {
            time_update_last = imu_next.stamp;
          }
          t_last = imu_next.stamp;
          input_in.gyro << imu_next.angular_vel(0), imu_next.angular_vel(1), imu_next.angular_vel(2);
          input_in.acc << imu_next.linear_acc(0), imu_next.linear_acc(1), imu_next.linear_acc(2);
          input_in.acc = input_in.acc * g_m_s2 / acc_norm;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        } else {
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
        }
      }
    }
  }
  (void)imu_prop_cov;
}

void OdometryEstimation::map_incremental() {
  PointVectorT points_to_add;
  const int cur_pts = static_cast<int>(feats_down_world->size());
  points_to_add.reserve(cur_pts);

  for (int i = 0; i < cur_pts; ++i) {
    PointT& point_world = feats_down_world->points[i];
    if (!nearest_points[i].empty()) {
      const PointVectorT& points_near = nearest_points[i];
      const Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < static_cast<int>(points_near.size()); readd_i++) {
        const Eigen::Vector3f dis2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          std::fabs(dis2_center.x()) < 0.5 * filter_size_map_min &&
          std::fabs(dis2_center.y()) < 0.5 * filter_size_map_min &&
          std::fabs(dis2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) points_to_add.emplace_back(point_world);
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox->AddPoints(points_to_add);
}

void OdometryEstimation::emit_outputs(const batch::MeasureGroup& measurements) {
  auto frame = std::make_shared<KeyFrame>();
  frame->id = frame_counter++;
  frame->stamp = measurements.lidar_end_time;
  frame->frame_id = FrameId::IMU;
  if (!use_imu_as_input) {
    frame->T_world_imu.linear() = kf_output.x_.rot;
    frame->T_world_imu.translation() = kf_output.x_.pos;
    frame->v_world_imu = kf_output.x_.vel;
    frame->imu_bias.head<3>() = kf_output.x_.bg;
    frame->imu_bias.tail<3>() = kf_output.x_.ba;
  } else {
    frame->T_world_imu.linear() = kf_input.x_.rot;
    frame->T_world_imu.translation() = kf_input.x_.pos;
    frame->v_world_imu = kf_input.x_.vel;
    frame->imu_bias.head<3>() = kf_input.x_.bg;
    frame->imu_bias.tail<3>() = kf_input.x_.ba;
  }

  // The KeyFrame always carries the IMU-frame scan: the extrinsic transform
  // is cheap relative to the batch optimization and lets every consumer
  // (rviz, backend optimization) work without extra gating.
  PointCloudT::Ptr cloud_body = boost::make_shared<PointCloudT>();
  cloud_body->resize(feats_undistort->size());
  // const Eigen::Matrix3d lidar_rotation = extrinsic_est_en
  //                                             ? (use_imu_as_input ? Eigen::Matrix3d(kf_input.x_.offset_R_L_I)
  //                                                                 : Eigen::Matrix3d(kf_output.x_.offset_R_L_I))
  //                                             : extrinsic_R;
  // const Eigen::Vector3d lidar_translation =
  //     extrinsic_est_en ? (use_imu_as_input ? Eigen::Vector3d(kf_input.x_.offset_T_L_I)
  //                                           : Eigen::Vector3d(kf_output.x_.offset_T_L_I))
  //                      : extrinsic_T;
  const Eigen::Matrix3d lidar_rotation = extrinsic_R;
  const Eigen::Vector3d lidar_translation = extrinsic_T;
  for (std::size_t index = 0; index < feats_undistort->size(); ++index) {
    const auto& input = feats_undistort->points[index];
    auto& output = cloud_body->points[index];
    const Eigen::Vector3d transformed = lidar_rotation * Eigen::Vector3d(input.x, input.y, input.z) + lidar_translation;
    output.x = transformed.x();
    output.y = transformed.y();
    output.z = transformed.z();
    output.intensity = input.intensity;
  }
  frame->cloud_imu = cloud_body;

  if (pcd_save_en) *map_cloud += *feats_down_world;

  Callbacks::on_new_frame(frame);

  logger->debug(
    "[batchlio] scan stamp={:.3f} feats_down={} eff={}",
    measurements.lidar_begin_time,
    feats_down_size,
    effct_feat_num);
}

void OdometryEstimation::point_body_to_world(const PointT& pi, PointT& po) {
  const Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
  Eigen::Vector3d p_global;
  // if (extrinsic_est_en) {
  //   if (!use_imu_as_input) {
  //     p_global =
  //         kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) + kf_output.x_.pos;
  //   } else {
  //     p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) + kf_input.x_.pos;
  //   }
  // } else {
  if (!use_imu_as_input) {
    p_global = kf_output.x_.rot * (extrinsic_R * p_body + extrinsic_T) + kf_output.x_.pos;
  } else {
    p_global = kf_input.x_.rot * (extrinsic_R * p_body + extrinsic_T) + kf_input.x_.pos;
  }
  // }
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

Eigen::Matrix<double, 24, 24> OdometryEstimation::process_noise_cov_input() {
  Eigen::Matrix<double, 24, 24> cov;
  cov.setZero();
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

Eigen::Matrix<double, 30, 30> OdometryEstimation::process_noise_cov_output() {
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

void OdometryEstimation::h_model_input(
  batch::state_input& s,
  Eigen::Matrix3d cov_p,
  Eigen::Matrix3d cov_r,
  esekfom::dyn_share_modified<double>& ekfom_data) {
  normvec->resize(time_seq[k_window]);
  int effect_num_k = 0;
  (void)cov_p;
  (void)cov_r;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) num_threads(omp_threads) reduction(+ : effect_num_k)
#endif
  for (int j = 0; j < time_seq[k_window]; j++) {
    VF(4) pabcd;
    pabcd.setZero();
    PointT& point_body_j = feats_down_body->points[idx + j + 1];
    PointT& point_world_j = feats_down_world->points[idx + j + 1];
    point_body_to_world(point_body_j, point_world_j);
    Eigen::Vector3d p_body = pbody_list[idx + j + 1];
    double p_norm = p_body.norm();
    Eigen::Vector3d p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto& points_near = nearest_points[idx + j + 1];
      ivox->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);
      if (points_near.size() < NUM_MATCH_POINTS) {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (batch::estimate_plane(pabcd, points_near, plane_thr)) {
          float pd2 =
            std::fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;
  for (int j = 0; j < time_seq[k_window]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      Eigen::Vector3d norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      // if (extrinsic_est_en) {
      //   Eigen::Vector3d p_body = pbody_list[idx + j + 1];
      //   Eigen::Matrix3d p_crossmat;
      //   Eigen::Matrix3d p_imu_crossmat;
      //   p_crossmat << SKEW_SYM_MATRX(p_body);
      //   Eigen::Vector3d point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
      //   p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
      //   Eigen::Vector3d C(s.rot.transpose() * norm_vec);
      //   Eigen::Vector3d A(p_imu_crossmat * C);
      //   Eigen::Vector3d B(p_crossmat * s.offset_R_L_I.transpose() * C);
      //   ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A),
      //   VEC_FROM_ARRAY(B),
      //       VEC_FROM_ARRAY(C);
      // } else {
      {
        Eigen::Matrix3d point_crossmat = crossmat_list[idx + j + 1];
        Eigen::Vector3d C(s.rot.transpose() * norm_vec);
        Eigen::Vector3d A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0;
      }
      // }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z - normvec->points[j].intensity;
      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

void OdometryEstimation::h_model_output(
  batch::state_output& s,
  Eigen::Matrix3d cov_p,
  Eigen::Matrix3d cov_r,
  esekfom::dyn_share_modified<double>& ekfom_data) {
  normvec->resize(time_seq[k_window]);
  int effect_num_k = 0;
  (void)cov_p;
  (void)cov_r;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) num_threads(omp_threads) reduction(+ : effect_num_k)
#endif
  for (int j = 0; j < time_seq[k_window]; j++) {
    VF(4) pabcd;
    pabcd.setZero();
    PointT& point_body_j = feats_down_body->points[idx + j + 1];
    PointT& point_world_j = feats_down_world->points[idx + j + 1];
    point_body_to_world(point_body_j, point_world_j);
    Eigen::Vector3d p_body = pbody_list[idx + j + 1];
    double p_norm = p_body.norm();
    Eigen::Vector3d p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto& points_near = nearest_points[idx + j + 1];
      ivox->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);
      if (points_near.size() < NUM_MATCH_POINTS) {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (batch::estimate_plane(pabcd, points_near, plane_thr)) {
          float pd2 =
            std::fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;
  for (int j = 0; j < time_seq[k_window]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      Eigen::Vector3d norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      // if (extrinsic_est_en) {
      //   Eigen::Vector3d p_body = pbody_list[idx + j + 1];
      //   Eigen::Matrix3d p_crossmat;
      //   Eigen::Matrix3d p_imu_crossmat;
      //   p_crossmat << SKEW_SYM_MATRX(p_body);
      //   Eigen::Vector3d point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
      //   p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
      //   Eigen::Vector3d C(s.rot.transpose() * norm_vec);
      //   Eigen::Vector3d A(p_imu_crossmat * C);
      //   Eigen::Vector3d B(p_crossmat * s.offset_R_L_I.transpose() * C);
      //   ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A),
      //   VEC_FROM_ARRAY(B),
      //       VEC_FROM_ARRAY(C);
      // } else {
      {
        Eigen::Matrix3d point_crossmat = crossmat_list[idx + j + 1];
        Eigen::Vector3d C(s.rot.transpose() * norm_vec);
        Eigen::Vector3d A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0;
      }
      // }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z - normvec->points[j].intensity;
      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

void OdometryEstimation::h_model_imu_output(batch::state_output& s, esekfom::dyn_share_modified<double>& ekfom_data) {
  std::memset(ekfom_data.satu_check, false, 6);
  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * g_m_s2 / acc_norm - s.acc - s.ba;
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov,
    imu_meas_acc_cov;
  if (check_satu) {
    if (std::fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;
      ekfom_data.z_IMU(0) = 0.0;
    }
    if (std::fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      ekfom_data.z_IMU(1) = 0.0;
    }
    if (std::fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      ekfom_data.z_IMU(2) = 0.0;
    }
    if (std::fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      ekfom_data.z_IMU(3) = 0.0;
    }
    if (std::fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      ekfom_data.z_IMU(4) = 0.0;
    }
    if (std::fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      ekfom_data.z_IMU(5) = 0.0;
    }
  }
}

void OdometryEstimation::process(const batch::MeasureGroup& measurements) {
  const bool initializing_imu_local = imu_preprocess->imu_need_init;
  if (!initializing_imu_local) {
    for (const auto& imu : measurements.imu) {
      if (imu->stamp > last_queued_imu_stamp) {
        imu_deque.push_back(imu);
        last_queued_imu_stamp = imu->stamp;
      }
    }
    if (measurements.imu_after_end && measurements.imu_after_end->stamp > last_queued_imu_stamp) {
      imu_deque.push_back(measurements.imu_after_end);
      last_queued_imu_stamp = measurements.imu_after_end->stamp;
    }
    if (!imu_deque.empty() && imu_next.stamp <= 0.0) {
      imu_next = *imu_deque.front();
      imu_last = imu_next;
    }
  } else if (measurements.imu_after_end) {
    imu_next = *measurements.imu_after_end;
    if (!measurements.imu.empty()) imu_last = *measurements.imu.back();
  }

  handle_first_scan(measurements);

  imu_preprocess->process(measurements, feats_undistort);
  downsample_and_window(measurements);

  if (!imu_preprocess->after_imu_init) {
    if (!imu_preprocess->imu_need_init) {
      Eigen::Vector3d tmp_gravity;
      if (imu_enabled) {
        const Eigen::Vector3d mean_acc = imu_preprocess->get_mean_acceleration();
        tmp_gravity = -mean_acc / mean_acc.norm() * g_m_s2;
      } else {
        tmp_gravity << gravity_init_vec(0), gravity_init_vec(1), gravity_init_vec(2);
        imu_preprocess->after_imu_init = true;
      }
      Eigen::Matrix3d rot_init;
      imu_preprocess->set_init(tmp_gravity, rot_init);
      kf_input.x_.rot = rot_init;
      kf_output.x_.rot = rot_init;
      kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
    } else {
      return;
    }
  }

  if (!init_map) {
    initialize_map(measurements);
    return;
  }

  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  nearest_points.resize(feats_down_size);

  crossmat_list.resize(feats_down_size);
  pbody_list.resize(feats_down_size);

  for (std::size_t i = 0; i < feats_down_body->size(); i++) {
    Eigen::Vector3d point_this(
      feats_down_body->points[i].x,
      feats_down_body->points[i].y,
      feats_down_body->points[i].z);
    pbody_list[i] = point_this;
    // if (!extrinsic_est_en) {
    point_this = extrinsic_R * point_this + extrinsic_T;
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    crossmat_list[i] = point_crossmat;
    // }
  }
  if (!use_imu_as_input) {
    run_output_loop(measurements);
  } else {
    run_input_loop(measurements);
  }

  if (feats_down_size > 4) map_incremental();

  emit_outputs(measurements);
}

void OdometryEstimation::h_model_input_trampoline(
  batch::state_input& s,
  Eigen::Matrix3d cov_p,
  Eigen::Matrix3d cov_r,
  esekfom::dyn_share_modified<double>& ekfom_data) {
  active->h_model_input(s, cov_p, cov_r, ekfom_data);
}

void OdometryEstimation::h_model_output_trampoline(
  batch::state_output& s,
  Eigen::Matrix3d cov_p,
  Eigen::Matrix3d cov_r,
  esekfom::dyn_share_modified<double>& ekfom_data) {
  active->h_model_output(s, cov_p, cov_r, ekfom_data);
}

void OdometryEstimation::h_model_imu_output_trampoline(
  batch::state_output& s,
  esekfom::dyn_share_modified<double>& ekfom_data) {
  active->h_model_imu_output(s, ekfom_data);
}

thread_local OdometryEstimation* OdometryEstimation::active = nullptr;

}  // namespace batchlio
}  // namespace asuka
