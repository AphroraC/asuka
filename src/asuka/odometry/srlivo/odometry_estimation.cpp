#include <asuka/odometry/srlivo/odometry_estimation.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

#include <boost/random/mersenne_twister.hpp>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace srlivo {

void OdometryEstimationParam::load(const std::shared_ptr<spdlog::logger> &logger, odometryOptions &odometry_options, mapOptions &map_options) {
  const Config odometry_config = load_config("config_odometry");

  point_buffer_capacity = odometry_config.param<int>("odometry", "point_buffer_capacity", point_buffer_capacity);
  imu_buffer_capacity = odometry_config.param<int>("odometry", "imu_buffer_capacity", imu_buffer_capacity);
  image_buffer_capacity = odometry_config.param<int>("odometry", "image_buffer_capacity", image_buffer_capacity);

  scan_line = odometry_config.param<int>("preprocess", "scan_line", scan_line);
  scan_rate = odometry_config.param<int>("preprocess", "scan_rate", scan_rate);
  blind = odometry_config.param<double>("preprocess", "blind", blind);
  point_filter_num = odometry_config.param<int>("preprocess", "point_filter_num", point_filter_num);

  acc_cov = odometry_config.param<double>("imu", "acc_cov", acc_cov);
  gyr_cov = odometry_config.param<double>("imu", "gyr_cov", gyr_cov);
  b_acc_cov = odometry_config.param<double>("imu", "b_acc_cov", b_acc_cov);
  b_gyr_cov = odometry_config.param<double>("imu", "b_gyr_cov", b_gyr_cov);
  gravity_acc = odometry_config.param<Eigen::Vector3d>("imu", "gravity_acc", gravity_acc);

  image_width = odometry_config.param<int>("camera", "image_width", image_width);
  image_height = odometry_config.param<int>("camera", "image_height", image_height);
  camera_intrinsic = odometry_config.param_cast<Eigen::Matrix3d>("camera", "intrinsic");
  camera_dist_coeffs = odometry_config.param_cast<Eigen::Matrix<double, 5, 1>>("camera", "dist_coeffs");
  R_imu_camera = odometry_config.param_cast<Eigen::Matrix3d>("camera", "extrinsic_R_imu_camera");
  t_imu_camera = odometry_config.param_cast<Eigen::Vector3d>("camera", "extrinsic_t_imu_camera");

  R_imu_lidar = odometry_config.param_cast<Eigen::Matrix3d>("sensor", "extrinsic_R_imu_lidar");
  t_imu_lidar = odometry_config.param_cast<Eigen::Vector3d>("sensor", "extrinsic_t_imu_lidar");

  odometry_options.init_voxel_size = odometry_config.param<double>("odometry_options", "init_voxel_size", odometry_options.init_voxel_size);
  odometry_options.init_sample_voxel_size = odometry_config.param<double>("odometry_options", "init_sample_voxel_size", odometry_options.init_sample_voxel_size);
  odometry_options.init_num_frames = odometry_config.param<int>("odometry_options", "init_num_frames", odometry_options.init_num_frames);
  odometry_options.voxel_size = odometry_config.param<double>("odometry_options", "voxel_size", odometry_options.voxel_size);
  odometry_options.sample_voxel_size = odometry_config.param<double>("odometry_options", "sample_voxel_size", odometry_options.sample_voxel_size);
  odometry_options.max_distance = odometry_config.param<double>("odometry_options", "max_distance", odometry_options.max_distance);
  odometry_options.max_num_points_in_voxel = odometry_config.param<int>("odometry_options", "max_num_points_in_voxel", odometry_options.max_num_points_in_voxel);
  odometry_options.min_distance_points = odometry_config.param<double>("odometry_options", "min_distance_points", odometry_options.min_distance_points);
  odometry_options.distance_error_threshold = odometry_config.param<double>("odometry_options", "distance_error_threshold", odometry_options.distance_error_threshold);

  const std::string motion_compensation =
      odometry_config.param<std::string>("odometry_options", "motion_compensation", "CONSTANT_VELOCITY");
  if (motion_compensation == "IMU") odometry_options.motion_compensation = IMU;
  else if (motion_compensation == "CONSTANT_VELOCITY") odometry_options.motion_compensation = CONSTANT_VELOCITY;
  else logger->warn("unsupported motion_compensation {}, using CONSTANT_VELOCITY", motion_compensation);

  const std::string initialization =
      odometry_config.param<std::string>("odometry_options", "initialization", "INIT_IMU");
  if (initialization == "INIT_IMU") odometry_options.initialization = INIT_IMU;
  else if (initialization == "INIT_CONSTANT_VELOCITY") odometry_options.initialization = INIT_CONSTANT_VELOCITY;
  else logger->warn("unsupported initialization {}, using INIT_IMU", initialization);

  icpOptions &icp = odometry_options.optimize_options;
  icp.threshold_voxel_occupancy = odometry_config.param<int>("icp_options", "threshold_voxel_occupancy", icp.threshold_voxel_occupancy);
  icp.size_voxel_map = odometry_config.param<double>("icp_options", "size_voxel_map", icp.size_voxel_map);
  icp.num_iters_icp = odometry_config.param<int>("icp_options", "num_iters_icp", icp.num_iters_icp);
  icp.min_number_neighbors = odometry_config.param<int>("icp_options", "min_number_neighbors", icp.min_number_neighbors);
  icp.voxel_neighborhood = odometry_config.param<int>("icp_options", "voxel_neighborhood", icp.voxel_neighborhood);
  icp.power_planarity = odometry_config.param<double>("icp_options", "power_planarity", icp.power_planarity);
  icp.estimate_normal_from_neighborhood = odometry_config.param<bool>("icp_options", "estimate_normal_from_neighborhood", icp.estimate_normal_from_neighborhood);
  icp.max_number_neighbors = odometry_config.param<int>("icp_options", "max_number_neighbors", icp.max_number_neighbors);
  icp.max_dist_to_plane_icp = odometry_config.param<double>("icp_options", "max_dist_to_plane_icp", icp.max_dist_to_plane_icp);
  icp.threshold_orientation_norm = odometry_config.param<double>("icp_options", "threshold_orientation_norm", icp.threshold_orientation_norm);
  icp.threshold_translation_norm = odometry_config.param<double>("icp_options", "threshold_translation_norm", icp.threshold_translation_norm);
  icp.max_num_residuals = odometry_config.param<int>("icp_options", "max_num_residuals", icp.max_num_residuals);
  icp.min_num_residuals = odometry_config.param<int>("icp_options", "min_num_residuals", icp.min_num_residuals);
  icp.num_closest_neighbors = odometry_config.param<int>("icp_options", "num_closest_neighbors", icp.num_closest_neighbors);
  icp.weight_alpha = odometry_config.param<double>("icp_options", "weight_alpha", icp.weight_alpha);
  icp.weight_neighborhood = odometry_config.param<double>("icp_options", "weight_neighborhood", icp.weight_neighborhood);
  icp.debug_print = odometry_config.param<bool>("icp_options", "debug_print", icp.debug_print);

  map_options.size_voxel_map = odometry_config.param<double>("map_options", "size_voxel_map", map_options.size_voxel_map);
  map_options.max_num_points_in_voxel = odometry_config.param<int>("map_options", "max_num_points_in_voxel", map_options.max_num_points_in_voxel);
  map_options.min_distance_points = odometry_config.param<double>("map_options", "min_distance_points", map_options.min_distance_points);
  map_options.add_point_step = odometry_config.param<int>("map_options", "add_point_step", map_options.add_point_step);
  map_options.pub_point_minimum_views = odometry_config.param<int>("map_options", "pub_point_minimum_views", map_options.pub_point_minimum_views);

  const Config ros_config = load_config("config_ros");
  save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);
}

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");

  param.load(logger, odometry_options, map_options);

  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();
  cloud_preprocess_impl->set_num_scans(param.scan_line);
  cloud_preprocess_impl->set_scan_rate(param.scan_rate);
  cloud_preprocess_impl->set_blind(param.blind);
  cloud_preprocess_impl->set_point_filter_num(param.point_filter_num);

  eskf_pro = std::make_unique<eskfEstimator>();
  eskf_pro->setAccCov(param.acc_cov);
  eskf_pro->setGyrCov(param.gyr_cov);
  eskf_pro->setBiasAccCov(param.b_acc_cov);
  eskf_pro->setBiasGyrCov(param.b_gyr_cov);
  G = param.gravity_acc;
  G_norm = G.norm();
  eskf_pro->setGravityNorm(G_norm);

  img_pro = std::make_unique<imageProcessing>();
  img_pro->setImageWidth(param.image_width);
  img_pro->setImageHeight(param.image_height);
  std::vector<double> intrinsic_vec(9);
  for (int i = 0; i < 9; i++) intrinsic_vec[i] = param.camera_intrinsic.data()[i];
  std::vector<double> dist_vec(param.camera_dist_coeffs.data(), param.camera_dist_coeffs.data() + 5);
  img_pro->setCameraIntrinsic(intrinsic_vec);
  img_pro->setCameraDistCoeffs(dist_vec);
  img_pro->setExtrinR(param.R_imu_camera);
  img_pro->setExtrinT(param.t_imu_camera);

  point_buffer.set_capacity(param.point_buffer_capacity);
  imu_buffer.set_capacity(param.imu_buffer_capacity);
  img_buffer.set_capacity(param.image_buffer_capacity);
  img_time_buffer.set_capacity(param.image_buffer_capacity);

  R_imu_lidar = param.R_imu_lidar;
  t_imu_lidar = param.t_imu_lidar;
  R_imu_camera = param.R_imu_camera;
  t_imu_camera = param.t_imu_camera;

  last_time_lidar = -1.0;
  last_time_imu = -1.0;
  last_time_img = -1.0;
  last_get_measurement = -1.0;
  last_time_frame = -1.0;
  current_time = -1.0;
  dt_sum = 0.0;

  index_frame = 1;

  odometry_options.optimize_options.init_num_frames = odometry_options.init_num_frames;

  img_pro->printParameter();

  logger->warn("srlivo requires an image stream: no measurement window is produced until images arrive");
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr &imu) {
  if (kill_switch) return;
  imu_buffer.push_back(imu);
  last_time_imu = imu->stamp;

  if (last_get_measurement < 0)
    last_get_measurement = last_time_imu;
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr &cloud) {
  if (kill_switch) return;
  assert(stamp > last_time_lidar);
  points_per_frame = 0.9 * points_per_frame + 0.1 * static_cast<double>(cloud->size());
  for (const auto &pt : cloud->points) {
    point3D point_temp;
    point_temp.raw_point = Eigen::Vector3d(pt.x, pt.y, pt.z);
    point_temp.point = point_temp.raw_point;
    point_temp.relative_time = pt.curvature;                // ms
    point_temp.timestamp = pt.curvature * 1e-3 + stamp;     // s
    point_temp.alpha_time = 0.0;
    point_buffer.push_back(point_temp);
  }
  last_time_lidar = stamp;
}

void OdometryEstimation::insert_image(const ImageData::ConstPtr &image) {
  if (kill_switch) return;
  assert(image->stamp > last_time_img);
  img_buffer.push_back(image);
  img_time_buffer.push_back(image->stamp);
  last_time_img = image->stamp;
}

int OdometryEstimation::workload() const {
  // Lidar-side pending only, expressed in frame equivalents: the continuous
  // point stream sits above the rosbag gate's threshold between windows by
  // design (windows drain it at image rate), so the raw point count would
  // throttle the feeder to its 500 ms no-progress escape on every frame.
  const int frame_equivalent = static_cast<int>(point_buffer.size() / std::max(1.0, points_per_frame));
  return frame_equivalent;
}

void OdometryEstimation::stop() {
  kill_switch = true;
  point_buffer.clear();
  imu_buffer.clear();
  img_buffer.clear();
  img_time_buffer.clear();
  for (auto *frame : all_cloud_frame) {
    frame->release();
    delete frame;
  }
  all_cloud_frame.clear();
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  if (!param.save_map_en) return nullptr;

  // Colored map points (upstream saveColorPoints). Upstream iterates the
  // registered rgb points backwards and silently skips index 0; this port
  // walks the whole vector so every colored point is exported. The rgb
  // triplet stored in rgbPoint is BGR-ordered (raw cv::Vec3b), so the pack
  // below swaps channels exactly like the upstream publishers.
  PointCloudT::Ptr out(new PointCloudT());
  const auto &rgb_vec = img_pro->map_tracker->rgb_points_vec;
  for (size_t i = 0; i < rgb_vec.size(); i++) {
    const rgbPoint *p_point = rgb_vec[i];
    if (p_point->N_rgb < map_options.pub_point_minimum_views) continue;

    PointT pt;
    pt.x = p_point->getPosition()[0];
    pt.y = p_point->getPosition()[1];
    pt.z = p_point->getPosition()[2];
    pt.intensity = 0.0f;
    const int r = static_cast<int>(p_point->getRgb()[2]);
    const int g = static_cast<int>(p_point->getRgb()[1]);
    const int b = static_cast<int>(p_point->getRgb()[0]);
    const uint32_t packed = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
    std::memcpy(&pt.rgb, &packed, sizeof(pt.rgb));
    pt.curvature = 0.0f;
    out->points.push_back(pt);
  }
  return out;
}

std::shared_ptr<asuka::CloudPreprocess> OdometryEstimation::cloud_preprocess() {
  return cloud_preprocess_impl;
}

std::vector<Measurements> OdometryEstimation::getMeasurements() {
  std::vector<Measurements> measurements;

  while (true) {
    if (imu_buffer.empty() || img_buffer.empty() || img_time_buffer.empty() || point_buffer.empty())
      return measurements;

    if (!(point_buffer.back().timestamp > img_time_buffer.front())) {
      return measurements;
    }

    if (!(point_buffer.front().timestamp < img_time_buffer.front())) {
      // stale image: the point stream has moved past it
      img_time_buffer.pop_front();
      img_buffer.pop_front();
      continue;
    }

    if (!(imu_buffer.back()->stamp > img_time_buffer.front())) {
      return measurements;
    }

    if (!(imu_buffer.front()->stamp < img_time_buffer.front())) {
      img_time_buffer.pop_front();
      img_buffer.pop_front();
      continue;
    }

    Measurements measurement;

    if (last_get_measurement + cloud_preprocess_impl->get_sweep_interval() < img_time_buffer.front() - 0.5 * cloud_preprocess_impl->get_sweep_interval()) {
      measurement.time_image = last_get_measurement + cloud_preprocess_impl->get_sweep_interval();

      while (imu_buffer.front()->stamp < last_get_measurement + cloud_preprocess_impl->get_sweep_interval()) {
        measurement.imu_measurements.emplace_back(imu_buffer.front());
        imu_buffer.pop_front();
      }

      measurement.imu_measurements.emplace_back(imu_buffer.front());

      while (point_buffer.front().timestamp < last_get_measurement + cloud_preprocess_impl->get_sweep_interval()) {
        measurement.lidar_points.push_back(point_buffer.front());
        point_buffer.pop_front();
      }

      measurement.time_sweep.first = last_get_measurement;
      measurement.time_sweep.second = cloud_preprocess_impl->get_sweep_interval();

      measurement.rendering = false;

      if (measurement.lidar_points.size() > 0) {
        measurements.emplace_back(measurement);
      }

      last_get_measurement = last_get_measurement + cloud_preprocess_impl->get_sweep_interval();

      break;
    } else {
      measurement.time_image = img_time_buffer.front();
      measurement.image = img_buffer.front();

      img_time_buffer.pop_front();
      img_buffer.pop_front();

      while (imu_buffer.front()->stamp < measurement.time_image) {
        measurement.imu_measurements.emplace_back(imu_buffer.front());
        imu_buffer.pop_front();
      }

      measurement.imu_measurements.emplace_back(imu_buffer.front());

      while (point_buffer.front().timestamp < measurement.time_image) {
        measurement.lidar_points.push_back(point_buffer.front());
        point_buffer.pop_front();
      }

      measurement.time_sweep.first = last_get_measurement;
      measurement.time_sweep.second = measurement.time_image - last_get_measurement;

      measurement.rendering = true;

      if (measurement.lidar_points.size() > 0) {
        measurements.emplace_back(measurement);
      }

      last_get_measurement = measurement.time_image;

      break;
    }
  }

  return measurements;
}

void OdometryEstimation::runMeasurements(std::vector<Measurements> &measurements) {
  for (auto &measurement : measurements) {
    double time_frame = measurement.time_image;
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

    if (!initial_flag) {
      for (auto &imu_msg : measurement.imu_measurements) {
        double time_imu = imu_msg->stamp;

        if (time_imu <= time_frame) {
          current_time = time_imu;
          dx = imu_msg->linear_acc.x();
          dy = imu_msg->linear_acc.y();
          dz = imu_msg->linear_acc.z();
          rx = imu_msg->angular_vel.x();
          ry = imu_msg->angular_vel.y();
          rz = imu_msg->angular_vel.z();

          imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
        } else {
          double dt_1 = time_frame - current_time;
          double dt_2 = time_imu - time_frame;
          current_time = time_frame;
          assert(dt_1 >= 0);
          assert(dt_2 >= 0);
          assert(dt_1 + dt_2 > 0);
          double w1 = dt_2 / (dt_1 + dt_2);
          double w2 = dt_1 / (dt_1 + dt_2);
          dx = w1 * dx + w2 * imu_msg->linear_acc.x();
          dy = w1 * dy + w2 * imu_msg->linear_acc.y();
          dz = w1 * dz + w2 * imu_msg->linear_acc.z();
          rx = w1 * rx + w2 * imu_msg->angular_vel.x();
          ry = w1 * ry + w2 * imu_msg->angular_vel.y();
          rz = w1 * rz + w2 * imu_msg->angular_vel.z();

          imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
        }
      }
      eskf_pro->tryInit(imu_meas);
      initial_flag = eskf_pro->is_initialized();
      imu_meas.clear();

      last_time_frame = time_frame;

      std::vector<point3D>().swap(measurement.lidar_points);

      continue;
    }

    imuState imu_state_temp;

    imu_state_temp.timestamp = current_time;

    imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (eskf_pro->getLastAcc() - eskf_pro->getBa());
    imu_state_temp.un_gyr = eskf_pro->getLastGyr() - eskf_pro->getBg();
    imu_state_temp.trans = eskf_pro->getTranslation();
    imu_state_temp.quat = eskf_pro->getRotation();
    imu_state_temp.vel = eskf_pro->getVelocity();

    imu_states.push_back(imu_state_temp);

    for (auto &imu_msg : measurement.imu_measurements) {
      double time_imu = imu_msg->stamp;

      if (time_imu <= time_frame) {
        double dt = time_imu - current_time;

        if (dt < -1e-6) continue;
        assert(dt >= 0);
        current_time = time_imu;
        dx = imu_msg->linear_acc.x();
        dy = imu_msg->linear_acc.y();
        dz = imu_msg->linear_acc.z();
        rx = imu_msg->angular_vel.x();
        ry = imu_msg->angular_vel.y();
        rz = imu_msg->angular_vel.z();

        imuState imu_state_temp;

        imu_state_temp.timestamp = current_time;

        imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
        imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

        dt_sum = dt_sum + dt;
        eskf_pro->predict(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

        imu_state_temp.trans = eskf_pro->getTranslation();
        imu_state_temp.quat = eskf_pro->getRotation();
        imu_state_temp.vel = eskf_pro->getVelocity();

        imu_states.push_back(imu_state_temp);
      } else {
        double dt_1 = time_frame - current_time;
        double dt_2 = time_imu - time_frame;
        current_time = time_frame;
        assert(dt_1 >= 0);
        assert(dt_2 >= 0);
        assert(dt_1 + dt_2 > 0);
        double w1 = dt_2 / (dt_1 + dt_2);
        double w2 = dt_1 / (dt_1 + dt_2);
        dx = w1 * dx + w2 * imu_msg->linear_acc.x();
        dy = w1 * dy + w2 * imu_msg->linear_acc.y();
        dz = w1 * dz + w2 * imu_msg->linear_acc.z();
        rx = w1 * rx + w2 * imu_msg->angular_vel.x();
        ry = w1 * ry + w2 * imu_msg->angular_vel.y();
        rz = w1 * rz + w2 * imu_msg->angular_vel.z();

        imuState imu_state_temp;

        imu_state_temp.timestamp = current_time;

        imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
        imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

        dt_sum = dt_sum + dt_1;
        eskf_pro->predict(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

        imu_state_temp.trans = eskf_pro->getTranslation();
        imu_state_temp.quat = eskf_pro->getRotation();
        imu_state_temp.vel = eskf_pro->getVelocity();

        imu_states.push_back(imu_state_temp);
      }
    }

    process(measurement.lidar_points, measurement.time_sweep.first, measurement.time_sweep.second, measurement.image, measurement.rendering);

    imu_states.clear();

    last_time_frame = time_frame;
    index_frame++;
  }
}

bool OdometryEstimation::process_once() {
  if (kill_switch) return false;

  std::vector<Measurements> measurements = getMeasurements();
  if (measurements.empty()) {
    // The upstream main loop simply spins: without images no measurement
    // window can ever be assembled. Warn (throttled) when point data keeps
    // piling up without an image stream to anchor windows.
    if (!img_buffer.empty() || point_buffer.empty()) return false;
    if (++missing_image_warn_count % 2000 == 1) {
      logger->warn("no measurement window: waiting for images ({} lidar points buffered)", point_buffer.size());
    }
    return false;
  }

  runMeasurements(measurements);
  return true;
}

void OdometryEstimation::makePointTimestamp(std::vector<point3D> &sweep, double time_begin, double time_end) {
  if (point_time_enable) {
    double delta_t = time_end - time_begin;

    for (int i = 0; i < sweep.size(); i++) {
      sweep[i].relative_time = sweep[i].timestamp - time_begin;
      sweep[i].alpha_time = sweep[i].relative_time / delta_t;
      sweep[i].relative_time = sweep[i].relative_time * 1000.0;
      if (sweep[i].alpha_time > 1.0) sweep[i].alpha_time = 1.0 - 1e-5;
    }
  } else {
    double delta_t = time_end - time_begin;

    std::vector<point3D>::iterator iter = sweep.begin();

    while (iter != sweep.end()) {
      if ((*iter).timestamp > time_end) iter = sweep.erase(iter);
      else if ((*iter).timestamp < time_begin) iter = sweep.erase(iter);
      else {
        (*iter).relative_time = (*iter).timestamp - time_begin;
        (*iter).alpha_time = (*iter).relative_time / delta_t;
        (*iter).relative_time = (*iter).relative_time * 1000.0;
        iter++;
      }
    }
  }
}

cloudFrame *OdometryEstimation::buildFrame(std::vector<point3D> &cut_sweep, state *cur_state, double timestamp_begin, double timestamp_offset) {
  std::vector<point3D> frame(cut_sweep);

  double offset_begin = 0;
  double offset_end = timestamp_offset;

  double time_sweep_begin = timestamp_begin;
  double time_frame_begin = timestamp_begin;

  makePointTimestamp(frame, time_frame_begin, timestamp_begin + timestamp_offset);
  if (odometry_options.motion_compensation == CONSTANT_VELOCITY)
    distortFrameByConstant(frame, imu_states, time_frame_begin, R_imu_lidar, t_imu_lidar);
  else if (odometry_options.motion_compensation == IMU)
    distortFrameByImu(frame, imu_states, time_frame_begin, R_imu_lidar, t_imu_lidar);
  double sample_size = index_frame < odometry_options.init_num_frames ? odometry_options.init_voxel_size : odometry_options.voxel_size;
  boost::mt19937_64 seed;
  std::shuffle(frame.begin(), frame.end(), seed);
  if (odometry_options.voxel_size > 0) {
    subSampleFrame(frame, sample_size);
    std::shuffle(frame.begin(), frame.end(), seed);
  }
  transformAllImuPoint(frame, imu_states, R_imu_lidar, t_imu_lidar);
  double dt_offset = 0;
  if (index_frame > 1)
    dt_offset -= time_frame_begin - all_cloud_frame.back()->time_sweep_end;
  if (index_frame <= 2) {
    for (auto &point_temp : frame) {
      point_temp.alpha_time = 1.0;
    }
  }
  if (index_frame > 2) {
    for (auto &point_temp : frame) {
      transformPoint(point_temp, cur_state->rotation, cur_state->translation, R_imu_lidar, t_imu_lidar);
    }
  } else {
    for (auto &point_temp : frame) {
      Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
      Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();
      transformPoint(point_temp, q_identity, t_zero, R_imu_lidar, t_imu_lidar);
    }
  }
  cloudFrame *p_frame = new cloudFrame(frame, cur_state);
  p_frame->time_sweep_begin = time_sweep_begin;
  p_frame->time_sweep_end = timestamp_begin + timestamp_offset;
  p_frame->time_frame_begin = time_frame_begin;
  p_frame->time_frame_end = p_frame->time_sweep_end;
  p_frame->offset_begin = offset_begin;
  p_frame->offset_end = offset_end;
  p_frame->dt_offset = dt_offset;
  p_frame->id = all_cloud_frame.size();
  p_frame->sub_id = 0;
  p_frame->frame_id = index_frame;
  all_cloud_frame.push_back(p_frame);
  return p_frame;
}

void OdometryEstimation::stateInitialization(state *cur_state) {
  if (index_frame <= 2) {
    cur_state->rotation = Eigen::Quaterniond::Identity();
    cur_state->translation = Eigen::Vector3d::Zero();
  } else if (index_frame == 3) {
    if (odometry_options.initialization == INIT_CONSTANT_VELOCITY) {
      Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
              all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
      Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation +
                                   all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                                   all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() *
                                   (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation -
                                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);
      cur_state->rotation = q_next_end;
      cur_state->translation = t_next_end;
    } else if (odometry_options.initialization == INIT_IMU) {
      if (initial_flag) {
        cur_state->rotation = eskf_pro->getRotation();
        cur_state->translation = eskf_pro->getTranslation();
      } else {
        Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
        Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation +
                                     all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                                     all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() *
                                     (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation -
                                      all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);
        cur_state->rotation = q_next_end;
        cur_state->translation = t_next_end;
      }
    } else {
      cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
      cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
    }
  } else {
    if (odometry_options.initialization == INIT_CONSTANT_VELOCITY) {
      Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
              all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
      Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation +
                                   all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                                   all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() *
                                   (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation -
                                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);
      cur_state->rotation = q_next_end;
      cur_state->translation = t_next_end;
    } else if (odometry_options.initialization == INIT_IMU) {
      if (initial_flag) {
        cur_state->rotation = eskf_pro->getRotation();
        cur_state->translation = eskf_pro->getTranslation();
      } else {
        Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
        Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation +
                                     all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation *
                                     all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() *
                                     (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation -
                                      all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);
        cur_state->rotation = q_next_end;
        cur_state->translation = t_next_end;
      }
    } else {
      cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
      cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
    }
  }
}

optimizeSummary OdometryEstimation::stateEstimation(cloudFrame *p_frame, bool to_rendering) {
  icpOptions optimize_options = odometry_options.optimize_options;
  const double kSizeVoxelMap = optimize_options.size_voxel_map;
  const double kMinDistancePoints = odometry_options.min_distance_points;
  const int kMaxNumPointsInVoxel = odometry_options.max_num_points_in_voxel;

  optimizeSummary optimize_summary;

  if (p_frame->frame_id > 1) {
    double sample_voxel_size = p_frame->frame_id < odometry_options.init_num_frames ? odometry_options.init_sample_voxel_size : odometry_options.sample_voxel_size;

    optimize_summary = optimize(p_frame, optimize_options, sample_voxel_size);

    if (!optimize_summary.success) {
      return optimize_summary;
    }
  } else {
    p_frame->p_state->translation = eskf_pro->getTranslation();
    p_frame->p_state->rotation = eskf_pro->getRotation();
    p_frame->p_state->velocity = eskf_pro->getVelocity();
    p_frame->p_state->ba = eskf_pro->getBa();
    p_frame->p_state->bg = eskf_pro->getBg();
    G = eskf_pro->getGravity();
    G_norm = G.norm();
  }

  addPointsToMap(voxel_map, p_frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints, 0, to_rendering);

  return optimize_summary;
}

void OdometryEstimation::addPointToMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points,
                                       int min_num_points, cloudFrame *p_frame) {
  short kx = static_cast<short>(point.getPosition().x() / voxel_size);
  short ky = static_cast<short>(point.getPosition().y() / voxel_size);
  short kz = static_cast<short>(point.getPosition().z() / voxel_size);

  voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

  if (search != map.end()) {
    auto &voxel_block = (search.value());

    if (!voxel_block.IsFull()) {
      double sq_dist_min_to_points = 10 * voxel_size * voxel_size;

      for (int i(0); i < voxel_block.NumPoints(); ++i) {
        auto &_point = voxel_block.points[i];
        double sq_dist = (_point.getPosition() - point.getPosition()).squaredNorm();
        if (sq_dist < sq_dist_min_to_points) {
          sq_dist_min_to_points = sq_dist;
        }
      }
      if (sq_dist_min_to_points > (min_distance_points * min_distance_points)) {
        if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
          voxel_block.AddPoint(point);
        }
      }
    }
  } else {
    if (min_num_points <= 0) {
      voxelBlock voxel_block(max_num_points_in_voxel);
      voxel_block.AddPoint(point);
      map[voxel(kx, ky, kz)] = std::move(voxel_block);
    }
  }
}

void OdometryEstimation::addPointToColorMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points,
                                            int min_num_points, cloudFrame *p_frame, std::vector<voxelId> &voxels_recent_visited_temp) {
  bool add_point = true;
  int point_map_kx = static_cast<short>(point.getPosition().x() / min_distance_points);
  int point_map_ky = static_cast<short>(point.getPosition().y() / min_distance_points);
  int point_map_kz = static_cast<short>(point.getPosition().z() / min_distance_points);
  int kx = static_cast<short>(point.getPosition().x() / voxel_size);
  int ky = static_cast<short>(point.getPosition().y() / voxel_size);
  int kz = static_cast<short>(point.getPosition().z() / voxel_size);
  if (hashmap_3d_points.if_exist(point_map_kx, point_map_ky, point_map_kz))
    add_point = false;
  voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));
  if (search != map.end()) {
    auto &voxel_block = (search.value());
    if (!voxel_block.IsFull()) {
      if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
        voxel_block.AddPoint(point);
        if (add_point) {
          img_pro->map_tracker->mutex_rgb_points_vec->lock();
          point.point_index = img_pro->map_tracker->rgb_points_vec.size();
          img_pro->map_tracker->rgb_points_vec.push_back(&voxel_block.points.back());
          hashmap_3d_points.insert(point_map_kx, point_map_ky, point_map_kz, img_pro->map_tracker->rgb_points_vec.back());
          img_pro->map_tracker->mutex_rgb_points_vec->unlock();
        }
      }
    }
    if (fabs(p_frame->time_sweep_end - img_pro->time_last_process) > 1e-5 && fabs(voxel_block.last_visited_time - p_frame->time_sweep_end) > 1e-5) {
      voxel_block.last_visited_time = p_frame->time_sweep_end;
      voxels_recent_visited_temp.push_back(voxelId(kx, ky, kz));
    }
  } else {
    if (min_num_points <= 0) {
      voxelBlock voxel_block(max_num_points_in_voxel);
      voxel_block.AddPoint(point);
      map[voxel(kx, ky, kz)] = std::move(voxel_block);
      if (add_point) {
        img_pro->map_tracker->mutex_rgb_points_vec->lock();
        point.point_index = img_pro->map_tracker->rgb_points_vec.size();
        img_pro->map_tracker->rgb_points_vec.push_back(&map[voxel(kx, ky, kz)].points.back());
        hashmap_3d_points.insert(point_map_kx, point_map_ky, point_map_kz, img_pro->map_tracker->rgb_points_vec.back());
        img_pro->map_tracker->mutex_rgb_points_vec->unlock();
      }
      if (fabs(p_frame->time_sweep_end - img_pro->time_last_process) > 1e-5 && fabs(map[voxel(kx, ky, kz)].last_visited_time - p_frame->time_sweep_end) > 1e-5) {
        map[voxel(kx, ky, kz)].last_visited_time = p_frame->time_sweep_end;
        voxels_recent_visited_temp.push_back(voxelId(kx, ky, kz));
      }
    }
  }
}

void OdometryEstimation::addPointsToMap(voxelHashMap &map, cloudFrame *p_frame, double voxel_size, int max_num_points_in_voxel,
                                        double min_distance_points, int min_num_points, bool to_rendering) {
  if (to_rendering) {
    voxels_recent_visited_temp.clear();
    std::vector<voxelId>().swap(voxels_recent_visited_temp);
  }
  int number_of_voxels_before_add = voxels_recent_visited_temp.size();
  int point_idx = 0;
  for (const auto &point : p_frame->point_frame) {
    rgbPoint rgb_point(point.point);
    addPointToMap(map, rgb_point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points, p_frame);
    if (point_idx % map_options.add_point_step == 0)
      addPointToColorMap(color_voxel_map, rgb_point, map_options.size_voxel_map, map_options.max_num_points_in_voxel, map_options.min_distance_points, 0, p_frame, voxels_recent_visited_temp);
    point_idx++;
  }
  if (to_rendering) {
    img_pro->map_tracker->voxels_recent_visited.clear();
    std::vector<voxelId>().swap(img_pro->map_tracker->voxels_recent_visited);
    img_pro->map_tracker->voxels_recent_visited = voxels_recent_visited_temp;
    img_pro->map_tracker->number_of_new_visited_voxel = img_pro->map_tracker->voxels_recent_visited.size() - number_of_voxels_before_add;
  }
}

void OdometryEstimation::emitKeyFrame(cloudFrame *p_frame, bool rendering) {
  auto key_frame = std::make_shared<asuka::KeyFrame>();
  key_frame->id = keyframe_counter++;
    // KeyFrame::stamp contract: sweep start (see callbacks.hpp on_new_frame).
  key_frame->stamp = p_frame->time_sweep_begin;
  key_frame->frame_id = FrameId::IMU;
  key_frame->T_world_imu = Eigen::Isometry3d::Identity();
  key_frame->T_world_imu.linear() = p_frame->p_state->rotation.toRotationMatrix();
  key_frame->T_world_imu.translation() = p_frame->p_state->translation;
  key_frame->v_world_imu = p_frame->p_state->velocity;
  key_frame->imu_bias.head<3>() = p_frame->p_state->bg;
  key_frame->imu_bias.tail<3>() = p_frame->p_state->ba;

  PointCloudT::Ptr cloud(new PointCloudT());
  cloud->resize(p_frame->point_frame.size());
  for (size_t i = 0; i < p_frame->point_frame.size(); i++) {
    const auto &pt = p_frame->point_frame[i];
    auto &out = cloud->points[i];
    out.x = static_cast<float>(pt.imu_point.x());
    out.y = static_cast<float>(pt.imu_point.y());
    out.z = static_cast<float>(pt.imu_point.z());
    out.intensity = 0.0f;
    out.curvature = static_cast<float>(pt.relative_time);  // ms
    out.rgb = 0.0f;
    if (rendering && p_frame->frame_id > 2) {
      // fresh per-window coloring: project the world-frame point into this
      // window's image (pt.point carries the world position after optimize)
      double u = 0.0, v = 0.0;
      pcl::PointXYZRGB rgb_pt;
      if (p_frame->project3dPointInThisImage(pt.point, u, v, &rgb_pt, 1.0)) {
        const uint32_t packed = (static_cast<uint32_t>(rgb_pt.r) << 16) | (static_cast<uint32_t>(rgb_pt.g) << 8) | static_cast<uint32_t>(rgb_pt.b);
        std::memcpy(&out.rgb, &packed, sizeof(out.rgb));
      }
    }
  }
  key_frame->cloud_imu = cloud;

  Callbacks::on_new_frame(key_frame);
}

void OdometryEstimation::process(std::vector<point3D> &cut_sweep, double timestamp_begin, double timestamp_offset, const ImageData::ConstPtr &cur_image, bool to_rendering) {
  state *cur_state = new state();

  stateInitialization(cur_state);

  std::vector<point3D> const_frame;

  const_frame.insert(const_frame.end(), cut_sweep.begin(), cut_sweep.end());

  cloudFrame *p_frame = buildFrame(const_frame, cur_state, timestamp_begin, timestamp_offset);

  optimizeSummary summary = stateEstimation(p_frame, to_rendering);

  dt_sum = 0;

  if (all_cloud_frame.size() < 3) {
    p_frame->p_state->fx = img_pro->getCameraIntrinsic()(0, 0);
    p_frame->p_state->fy = img_pro->getCameraIntrinsic()(1, 1);
    p_frame->p_state->cx = img_pro->getCameraIntrinsic()(0, 2);
    p_frame->p_state->cy = img_pro->getCameraIntrinsic()(1, 2);

    p_frame->p_state->R_imu_camera = R_imu_camera;
    p_frame->p_state->t_imu_camera = t_imu_camera;
  } else {
    p_frame->p_state->fx = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->fx;
    p_frame->p_state->fy = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->fy;
    p_frame->p_state->cx = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->cx;
    p_frame->p_state->cy = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->cy;

    p_frame->p_state->R_imu_camera = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->R_imu_camera;
    p_frame->p_state->t_imu_camera = all_cloud_frame[all_cloud_frame.size() - 2]->p_state->t_imu_camera;
  }

  p_frame->p_state->q_world_camera = Eigen::Quaterniond(p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->R_imu_camera);
  p_frame->p_state->t_world_camera = p_frame->p_state->rotation.toRotationMatrix() * p_frame->p_state->t_imu_camera + p_frame->p_state->translation;
  p_frame->refreshPoseForProjection();

  if (to_rendering) {
    p_frame->image_data = cur_image;
    p_frame->rgb_image = *cur_image->image;
    p_frame->image_cols = cur_image->image->cols;
    p_frame->image_rows = cur_image->image->rows;

    img_pro->process(color_voxel_map, p_frame);
  }

  emitKeyFrame(p_frame, to_rendering);

  int num_remove = 0;

  if (initial_flag) {
    if (index_frame > 1) {
      while (all_cloud_frame.size() > 2) {
        all_cloud_frame[0]->release();
        delete all_cloud_frame[0];
        all_cloud_frame.erase(all_cloud_frame.begin());
        num_remove++;
      }
      assert(all_cloud_frame.size() == 2);
    }
  } else {
    while (all_cloud_frame.size() > odometry_options.num_for_initialization) {
      all_cloud_frame[0]->release();
      delete all_cloud_frame[0];
      all_cloud_frame.erase(all_cloud_frame.begin());
      num_remove++;
    }
  }

  for (size_t i = 0; i < all_cloud_frame.size(); i++)
    all_cloud_frame[i]->id = all_cloud_frame[i]->id - num_remove;
}

}  // namespace srlivo
}  // namespace asuka
