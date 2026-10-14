#include <asuka/odometry/smallpointlio/odometry_estimation.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace smallpointlio {
namespace {

constexpr int num_match_points = 5;

}  // namespace

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");

  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config config = load_config("config_odometry");
  param.laser_point_cov = config.param<double>("filter", "laser_point_cov", 0.01);
  param.imu_meas_acc_cov = config.param<double>("filter", "imu_meas_acc_cov", 0.01);
  param.imu_meas_omg_cov = config.param<double>("filter", "imu_meas_omg_cov", 0.01);
  param.velocity_cov = config.param<double>("filter", "velocity_cov", 20.0);
  param.acceleration_cov = config.param<double>("filter", "acceleration_cov", 500.0);
  param.omg_cov = config.param<double>("filter", "omg_cov", 1000.0);
  param.ba_cov = config.param<double>("filter", "ba_cov", 0.0001);
  param.bg_cov = config.param<double>("filter", "bg_cov", 0.0001);
  param.plane_threshold = config.param<double>("filter", "plane_threshold", 0.1);
  param.match_squared = config.param<double>("filter", "match_squared", 81.0);
  param.map_resolution = config.param<float>("filter", "map_resolution", 0.5f);
  param.init_map_size = config.param<int>("filter", "init_map_size", 10);

  param.check_satu = config.param<bool>("imu", "check_satu", true);
  param.satu_acc = config.param<double>("imu", "satu_acc", 3.0) * 0.99;
  param.satu_gyro = config.param<double>("imu", "satu_gyro", 35.0) * 0.99;

  param.gravity = config.param<Eigen::Vector3d>("imu", "gravity", Eigen::Vector3d(0, 0, -9.81));
  param.fix_gravity_direction = config.param<bool>("imu", "fix_gravity_direction", true);
  param.imu_acceleration_scale = param.gravity.norm() / config.param<double>("imu", "acc_norm", 1.0);

  const float min_distance = config.param<float>("preprocess", "min_distance", 0.5f);
  const float max_distance = config.param<float>("preprocess", "max_distance", 1000.0f);
  param.min_distance_squared = min_distance * min_distance;
  param.max_distance_squared = max_distance * max_distance;
  param.point_filter_num = std::max(1, config.param<int>("preprocess", "point_filter_num", 1));
  param.space_downsample = config.param<bool>("preprocess", "space_downsample", false);
  param.space_downsample_leaf_size = config.param<float>("preprocess", "space_downsample_leaf_size", 0.5f);

  param.publish_odometry_without_downsample =
    config.param<bool>("publish", "publish_odometry_without_downsample", false);

  const Config ros_config = load_config("config_ros");
  param.save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);
  if (!config.has("sensor")) {
    throw std::runtime_error("config_odometry: no sensor block");
  }
  param.lidar_T_wrt_imu =
    config.param<Eigen::Vector3d>("sensor", "extrinsic_T", Eigen::Vector3d(-0.011, -0.02329, 0.04412));
  param.lidar_R_wrt_imu = config.param<Eigen::Matrix3d>("sensor", "extrinsic_R", Eigen::Matrix3d::Identity());

  kf.init(
    [this](const EskfState& s, PointMeasurementResult& measurement_result) { h_point(s, measurement_result); },
    [this](const EskfState& s, ImuMeasurementResult& measurement_result) { h_imu(s, measurement_result); });

  ivox = std::make_shared<::smallpointlio::IVox>(param.map_resolution, 1000000);
  kf.cov = Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM>::Identity() * 0.01;
  kf.cov.block<3, 3>(EskfState::gravity_index, EskfState::gravity_index).diagonal().fill(0.0001);
  kf.cov.block<3, 3>(EskfState::bg_index, EskfState::bg_index).diagonal().fill(0.001);
  kf.cov.block<3, 3>(EskfState::ba_index, EskfState::ba_index).diagonal().fill(0.001);

  process_noise = process_noise_cov();

  logger->info(
    "smallpointlio::OdometryEstimation initialized: gravity=({:.2f},{:.2f},{:.2f}) resolution={} lp_cov={} "
    "imu_acc_cov={} imu_omg_cov={}",
    param.gravity.x(),
    param.gravity.y(),
    param.gravity.z(),
    param.map_resolution,
    param.laser_point_cov,
    param.imu_meas_acc_cov,
    param.imu_meas_omg_cov);
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  imu_preprocess->insert_imu(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  dense_points.clear();
  dense_points.reserve(cloud->size());
  filtered_points.clear();
  filtered_points.reserve(cloud->size());

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const PointT& point = cloud->points[i];
    RawPoint raw;
    raw.timestamp = stamp + static_cast<double>(point.curvature) / 1000.0;
    raw.position = Eigen::Vector3f(point.x, point.y, point.z);

    if (raw.timestamp >= last_timestamp_dense_point) {
      dense_points.push_back(raw);
    }

    if (i % static_cast<std::size_t>(param.point_filter_num) != 0) continue;
    if (raw.timestamp < last_timestamp_lidar) continue;
    // Upstream does not guard against non-finite coordinates here; matching it
    // keeps the accepted point set identical.
    const float dist = raw.position.squaredNorm();
    if (dist < param.min_distance_squared || dist > param.max_distance_squared) continue;

    filtered_points.push_back(raw);
  }

  if (param.space_downsample) {
    downsampler.voxelgrid_sampling(filtered_points, processed_pointcloud, param.space_downsample_leaf_size);
  } else {
    processed_pointcloud = std::move(filtered_points);
  }

  std::sort(dense_points.begin(), dense_points.end(), [](const RawPoint& lhs, const RawPoint& rhs) {
    return lhs.timestamp < rhs.timestamp;
  });
  std::sort(processed_pointcloud.begin(), processed_pointcloud.end(), [](const RawPoint& lhs, const RawPoint& rhs) {
    return lhs.timestamp < rhs.timestamp;
  });

  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (!dense_points.empty()) {
      last_timestamp_dense_point = dense_points.back().timestamp;
      dense_point_deque.insert(dense_point_deque.end(), dense_points.begin(), dense_points.end());
    }
    if (!processed_pointcloud.empty()) {
      last_timestamp_lidar = processed_pointcloud.back().timestamp;
      point_deque.insert(point_deque.end(), processed_pointcloud.begin(), processed_pointcloud.end());
    }
  }
}

int OdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return static_cast<int>(point_deque.size());
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  if (!param.save_map_en) return nullptr;
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return global_map;
}

void OdometryEstimation::append_global_map(const Eigen::Vector3f& position) {
  PointT point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  global_map->push_back(point);
}

bool OdometryEstimation::process_once() {
  std::vector<KeyFrame::ConstPtr> outputs;
  bool progress = false;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (point_deque.empty() || imu_preprocess->imu_deque.empty()) return false;
    progress = handle_once(outputs);
  }
  for (const auto& output : outputs) {
    if (output) Callbacks::on_new_frame(output);
  }
  return progress;
}

bool OdometryEstimation::handle_once(std::vector<KeyFrame::ConstPtr>& outputs) {
  auto& imu_deque = imu_preprocess->imu_deque;

  if (!is_init) {
    if (
      (!point_deque.empty() || !imu_deque.empty()) && point_deque.size() >= static_cast<size_t>(param.init_map_size) &&
      (!param.fix_gravity_direction || imu_deque.size() >= 200)) {
      for (const auto& point : point_deque) {
        ivox->add_point(point.position);
        if (param.save_map_en) append_global_map(point.position);
      }

      if (param.fix_gravity_direction) {
        kf.x.gravity = Eigen::Matrix<EskfState::ValueType, 3, 1>::Zero();
        for (const auto& imu_msg : imu_deque) {
          kf.x.gravity += imu_msg->linear_acc.cast<EskfState::ValueType>();
        }
        EskfState::ValueType scale = -static_cast<EskfState::ValueType>(param.gravity.norm()) / kf.x.gravity.norm();
        kf.x.gravity *= scale;
      } else {
        kf.x.gravity = param.gravity.cast<EskfState::ValueType>();
      }
      kf.x.acceleration = -kf.x.gravity;

      if (point_deque.empty()) {
        time_current = imu_deque.back()->stamp;
      } else if (imu_deque.empty()) {
        time_current = point_deque.back().timestamp;
      } else {
        time_current = std::max(point_deque.back().timestamp, imu_deque.back()->stamp);
      }
      kf.init_timestamp(time_current);

      point_deque.clear();
      dense_point_deque.clear();
      imu_deque.clear();
      is_init = true;

      logger->info("System initialized at time={:.3f}", time_current);
      return true;
    }
    // Not enough data yet: report "no progress" so the executor releases the
    // worker instead of spinning on this call.
    return false;
  }

  bool is_publish_odometry = !imu_deque.empty() && !dense_point_deque.empty() && !point_deque.empty() &&
                             imu_deque.front()->stamp < point_deque.back().timestamp;

  bool consumed = false;
  while (!imu_deque.empty() && !dense_point_deque.empty() && !point_deque.empty()) {
    const RawPoint& point_lidar_frame = point_deque.front();
    const RawPoint& dense_point_lidar_frame = dense_point_deque.front();
    const ImuData::ConstPtr& imu_msg = imu_deque.front();

    if (
      dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp &&
      dense_point_lidar_frame.timestamp < imu_msg->stamp) {
      // collect imu frame pointcloud for the keyframe
      const Eigen::Matrix<EskfState::ValueType, 3, 1> dense_point_imu_frame =
        param.lidar_R_wrt_imu * dense_point_lidar_frame.position.cast<EskfState::ValueType>() + param.lidar_T_wrt_imu;
      pointcloud_imu_frame.emplace_back(dense_point_imu_frame.cast<float>());

      dense_point_deque.pop_front();
      consumed = true;
    } else if (point_lidar_frame.timestamp < imu_msg->stamp) {
      // point update
      if (point_lidar_frame.timestamp < time_current) {
        point_deque.pop_front();
        consumed = true;
        continue;
      }
      time_current = point_lidar_frame.timestamp;

      kf.predict_state(time_current);
      this->point_lidar_frame = point_lidar_frame.position;
      kf.update_point();

      if (param.publish_odometry_without_downsample) {
        publish_odometry(time_current, outputs);
      }

      ivox->add_point(point_odom_frame);
      if (param.save_map_en) append_global_map(point_odom_frame);
      point_deque.pop_front();
      consumed = true;
    } else {
      // imu update
      if (imu_msg->stamp < time_current) {
        imu_deque.pop_front();
        consumed = true;
        continue;
      }
      time_current = imu_msg->stamp;

      kf.predict_state(time_current);
      kf.predict_cov(time_current, process_noise);

      angular_velocity = imu_msg->angular_vel.cast<EskfState::ValueType>();
      linear_acceleration = imu_msg->linear_acc.cast<EskfState::ValueType>();
      kf.update_imu();

      imu_deque.pop_front();
      consumed = true;
    }
  }

  if (is_publish_odometry) {
    if (!pointcloud_imu_frame.empty()) {
      PointCloudT::Ptr cloud_imu = boost::make_shared<PointCloudT>();
      cloud_imu->resize(pointcloud_imu_frame.size());
      for (size_t i = 0; i < pointcloud_imu_frame.size(); ++i) {
        cloud_imu->points[i].x = pointcloud_imu_frame[i].x();
        cloud_imu->points[i].y = pointcloud_imu_frame[i].y();
        cloud_imu->points[i].z = pointcloud_imu_frame[i].z();
      }
      cloud_imu->width = cloud_imu->size();
      cloud_imu->height = 1;
      pointcloud_imu_frame.clear();
      pending_cloud_imu = cloud_imu;
    }

    if (!param.publish_odometry_without_downsample) {
      publish_odometry(pointcloud_imu_frame.empty() ? time_current : keyframe_window_begin, outputs);
    } else if (pending_cloud_imu) {
      auto frame = std::make_shared<KeyFrame>();
      frame->id = -1;
      frame->stamp = pointcloud_imu_frame.empty() ? time_current : keyframe_window_begin;
      frame->frame_id = FrameId::IMU;
      frame->T_world_imu = Eigen::Isometry3d::Identity();
      frame->T_world_imu.linear() = kf.x.rotation.cast<double>();
      frame->T_world_imu.translation() = kf.x.position.cast<double>();
      frame->v_world_imu = kf.x.velocity.cast<double>();
      frame->imu_bias.head<3>() = kf.x.bg.cast<double>();
      frame->imu_bias.tail<3>() = kf.x.ba.cast<double>();
      frame->cloud_imu = pending_cloud_imu;
      pending_cloud_imu.reset();
      outputs.push_back(frame);
    }
  }

  return consumed;
}

void OdometryEstimation::publish_odometry(double timestamp, std::vector<KeyFrame::ConstPtr>& outputs) {
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  T_world_imu.translation() = kf.x.position.cast<double>();
  T_world_imu.linear() = kf.x.rotation.cast<double>();

  auto frame = std::make_shared<KeyFrame>();
  frame->id = frame_counter++;
  frame->stamp = timestamp;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = T_world_imu;
  frame->v_world_imu = kf.x.velocity.cast<double>();
  frame->imu_bias.head<3>() = kf.x.bg.cast<double>();
  frame->imu_bias.tail<3>() = kf.x.ba.cast<double>();
  frame->cloud_imu = pending_cloud_imu;
  pending_cloud_imu.reset();

  outputs.push_back(frame);
}

void OdometryEstimation::h_point(const EskfState& s, PointMeasurementResult& result) {
  result.valid = false;

  Eigen::Matrix<EskfState::ValueType, 3, 1> point_imu_frame;
  // if (extrinsic_est_en) {
  //   point_imu_frame = s.offset_R_L_I * point_lidar_frame.cast<EskfState::ValueType>() + s.offset_T_L_I;
  // } else {
  point_imu_frame = param.lidar_R_wrt_imu * point_lidar_frame.cast<EskfState::ValueType>() + param.lidar_T_wrt_imu;
  // }
  point_odom_frame = (s.rotation * point_imu_frame + s.position).cast<float>();

  ivox->get_closest_point(point_odom_frame, nearest_points, num_match_points);
  if (nearest_points.size() != num_match_points) return;

  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : nearest_points) {
    centroid.noalias() += p;
  }
  centroid /= static_cast<float>(nearest_points.size());

  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const auto& p : nearest_points) {
    Eigen::Vector3f centered = p - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<float>(nearest_points.size() - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  Eigen::Vector3f normal = solver.eigenvectors().col(0);
  float d = -normal.dot(centroid);

  for (int j = 0; j < num_match_points; j++) {
    float point_distance = std::abs(normal.dot(nearest_points[j]) + d);
    if (point_distance > param.plane_threshold) return;
  }

  float point_distance = normal.dot(point_odom_frame) + d;
  if (point_lidar_frame.norm() <= static_cast<float>(param.match_squared) * point_distance * point_distance) return;

  result.laser_point_cov = static_cast<EskfState::ValueType>(param.laser_point_cov);

  // if (extrinsic_est_en) {
  //   Eigen::Matrix<EskfState::ValueType, 3, 1> normal0 = normal.cast<EskfState::ValueType>();
  //   Eigen::Matrix<EskfState::ValueType, 3, 1> C = s.rotation.transpose() * normal0;
  //   Eigen::Matrix<EskfState::ValueType, 3, 1> A, B;
  //   A.noalias() = point_imu_frame.cross(C);
  //   B.noalias() = point_lidar_frame.cast<EskfState::ValueType>().cross(s.offset_R_L_I.transpose() * C);
  //   result.h << normal0.transpose(), A.transpose(), B.transpose(), C.transpose();
  // } else {
  {
    Eigen::Matrix<EskfState::ValueType, 3, 1> normal0 = normal.cast<EskfState::ValueType>();
    Eigen::Matrix<EskfState::ValueType, 3, 1> a;
    a.noalias() = point_imu_frame.cross(s.rotation.transpose() * normal0);
    result.h << normal0.transpose(), a.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
  // }
  result.z = -point_distance;
  result.valid = true;
}

void OdometryEstimation::h_imu(const EskfState& s, ImuMeasurementResult& result) {
  std::memset(result.satu_check, false, 6);
  result.z.segment<3>(0) = angular_velocity - s.omg - s.bg;
  result.z.segment<3>(3) = linear_acceleration * param.imu_acceleration_scale - s.acceleration - s.ba;
  result.imu_meas_omg_cov = static_cast<EskfState::ValueType>(param.imu_meas_omg_cov);
  result.imu_meas_acc_cov = static_cast<EskfState::ValueType>(param.imu_meas_acc_cov);

  if (param.check_satu) {
    for (int i = 0; i < 3; i++) {
      if (std::abs(angular_velocity(i)) >= param.satu_gyro) {
        result.satu_check[i] = true;
        result.z(i) = 0.0;
      }
      if (std::abs(linear_acceleration(i)) >= param.satu_acc) {
        result.satu_check[i + 3] = true;
        result.z(i + 3) = 0.0;
      }
    }
  }
}

Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> OdometryEstimation::process_noise_cov() const {
  Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> cov =
    Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM>::Zero();
  cov.block<3, 3>(EskfState::velocity_index, EskfState::velocity_index)
    .diagonal()
    .fill(static_cast<EskfState::ValueType>(param.velocity_cov));
  cov.block<3, 3>(EskfState::omg_index, EskfState::omg_index)
    .diagonal()
    .fill(static_cast<EskfState::ValueType>(param.omg_cov));
  cov.block<3, 3>(EskfState::acceleration_index, EskfState::acceleration_index)
    .diagonal()
    .fill(static_cast<EskfState::ValueType>(param.acceleration_cov));
  cov.block<3, 3>(EskfState::bg_index, EskfState::bg_index)
    .diagonal()
    .fill(static_cast<EskfState::ValueType>(param.bg_cov));
  cov.block<3, 3>(EskfState::ba_index, EskfState::ba_index)
    .diagonal()
    .fill(static_cast<EskfState::ValueType>(param.ba_cov));
  return cov;
}

}  // namespace smallpointlio
}  // namespace asuka
