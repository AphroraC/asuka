#pragma once

#include <boost/circular_buffer.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/smallpointlio/cloud_preprocess.hpp>
#include <asuka/odometry/smallpointlio/eskf.hpp>
#include <asuka/odometry/smallpointlio/imu_preprocess.hpp>
#include <small_ivox/small_ivox.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace smallpointlio {

class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override = default;

  void stop() override {}
  void clear_buffers() override;
  PointCloudT::Ptr save_map() override;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) override;

  bool process_once() override;
  int workload() const override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override { return cloud_preprocess_impl; }

private:
  void handle_once(std::vector<KeyFrame::ConstPtr>& outputs);
  void publish_odometry(double timestamp, std::vector<KeyFrame::ConstPtr>& outputs);
  void h_point(const EskfState& s, PointMeasurementResult& result);
  void h_imu(const EskfState& s, ImuMeasurementResult& result);
  Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> process_noise_cov() const;

  Eskf kf;
  std::shared_ptr<::smallpointlio::IVox> ivox{nullptr};

  Eigen::Vector3d lidar_T_wrt_imu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d lidar_R_wrt_imu{Eigen::Matrix3d::Identity()};
  Eigen::Vector3f point_lidar_frame{Eigen::Vector3f::Zero()};
  Eigen::Vector3f point_odom_frame{Eigen::Vector3f::Zero()};
  double imu_acceleration_scale{1.0};
  Eigen::Matrix<EskfState::ValueType, 3, 1> angular_velocity{Eigen::Matrix<EskfState::ValueType, 3, 1>::Zero()};
  Eigen::Matrix<EskfState::ValueType, 3, 1> linear_acceleration{Eigen::Matrix<EskfState::ValueType, 3, 1>::Zero()};

  double laser_point_cov{0.01};
  double imu_meas_acc_cov{0.01};
  double imu_meas_omg_cov{0.01};
  double velocity_cov{20.0};
  double acceleration_cov{500.0};
  double omg_cov{1000.0};
  double ba_cov{0.0001};
  double bg_cov{0.0001};
  double plane_threshold{0.1};
  double match_squared{81.0};
  float map_resolution{0.5f};
  bool check_satu{true};
  double satu_acc{3.0};
  double satu_gyro{35.0};

  bool is_init{false};
  double time_current{0.0};
  Eigen::Vector3d gravity{Eigen::Vector3d::Zero()};
  bool fix_gravity_direction{true};
  int init_map_size{10};
  bool publish_odometry_without_downsample{false};
  std::vector<Eigen::Vector3f> nearest_points;

  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};
  boost::circular_buffer<RawPoint> point_deque{50000};
  boost::circular_buffer<RawPoint> dense_point_deque{50000};

  std::vector<Eigen::Vector3f> pointcloud_imu_frame;
  PointCloudT::Ptr pending_cloud_imu{nullptr};

  Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> process_noise;

  mutable std::mutex buffer_mutex;
  long frame_counter{0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace smallpointlio
}  // namespace asuka
