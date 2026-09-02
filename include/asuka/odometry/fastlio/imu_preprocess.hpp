#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fastlio/so3_math.hpp>
#include <fastlio/use_ikfom.hpp>
#include <fastlio/pose6d.hpp>
#include <fastlio/esekfom/esekfom.hpp>

#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/core/callbacks.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlio {

struct ImuMeasureGroup {
  ImuMeasureGroup() = default;
  double lidar_beg_time{0.0};
  double lidar_end_time{0.0};
  PointCloudT::ConstPtr lidar;
  std::vector<ImuData::ConstPtr> imu;
};

class ImuPreprocess : public asuka::ImuPreprocess {
public:
  ImuPreprocess();
  ~ImuPreprocess() override = default;

  void reset();

  void initialize(double lidar_start_time) override;

  void process(
    const ImuMeasureGroup& meas,
    esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state,
    PointCloudT::Ptr& pcl_out);

  bool is_initialized() const override { return !imu_need_init; }
  Eigen::Vector3d get_mean_acceleration() const override { return mean_acc; }
  Eigen::Vector3d get_mean_gyroscope() const override { return mean_gyro; }

private:
  void imu_init(const ImuMeasureGroup& meas, esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state);
  void undistort_pcl(
    const ImuMeasureGroup& meas,
    esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state,
    PointCloudT& pcl_out);

  static constexpr int max_ini_count = 10;
  static constexpr double g_m_s2 = 9.81;
  bool gravity_estimation{false};

  std::vector<ikfom::Pose6D> imu_poses;
  ImuData::ConstPtr last_imu{nullptr};

  Eigen::Vector3d mean_acc{0, 0, -1.0};
  Eigen::Vector3d mean_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angvel_last{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_s_last{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cov_acc{0.1, 0.1, 0.1};
  Eigen::Vector3d cov_gyro{0.1, 0.1, 0.1};
  Eigen::Vector3d cov_acc_scale{0.1, 0.1, 0.1};
  Eigen::Vector3d cov_gyro_scale{0.1, 0.1, 0.1};
  Eigen::Vector3d cov_bias_gyro{0.0001, 0.0001, 0.0001};
  Eigen::Vector3d cov_bias_acc{0.0001, 0.0001, 0.0001};

  Eigen::Vector3d lidar_T_wrt_imu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d lidar_R_wrt_imu{Eigen::Matrix3d::Identity()};

  bool imu_need_init{true};
  bool b_first_frame{true};
  double first_lidar_time{0.0};
  double last_lidar_end_time{0.0};
  int init_iter_num{1};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace fastlio
}  // namespace asuka
