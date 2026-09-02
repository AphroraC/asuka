#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/batchlio/common.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace batchlio {

class ImuPreprocess : public asuka::ImuPreprocess {
public:
  ImuPreprocess();
  ~ImuPreprocess() override = default;

  void reset();

  void initialize(double lidar_start_time) override;

  void process(const batch::MeasureGroup& meas, PointCloudT::Ptr& pcl_out);
  void set_init(Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot);

  bool is_initialized() const override { return !imu_need_init; }
  Eigen::Vector3d get_mean_acceleration() const override { return mean_acc; }
  Eigen::Vector3d get_mean_gyroscope() const override { return mean_gyro; }

  Eigen::Vector3d gravity_value{Eigen::Vector3d::Zero()};
  bool imu_enabled{true};
  bool imu_need_init{true};
  bool after_imu_init{false};

private:
  void imu_init(const batch::MeasureGroup& meas, int& n);

  Eigen::Matrix<double, 12, 12> state_cov{Eigen::Matrix<double, 12, 12>::Identity()};
  Eigen::Vector3d mean_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mean_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cov_gyr_scale = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  Eigen::Vector3d cov_vel_scale = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  bool b_first_frame{true};
  double time_last_scan{0.0};
  int init_iter_num{1};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace batchlio
}  // namespace asuka
