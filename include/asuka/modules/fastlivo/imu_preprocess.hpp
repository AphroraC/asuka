#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include <asuka/core/types.hpp>
#include <asuka/modules/fastlivo/common_lib.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlivo {

class ImuPreprocess {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuPreprocess();
  ~ImuPreprocess();

  void Reset();
  void set_extrinsic(const Eigen::Vector3d& transl, const Eigen::Matrix3d& rot);
  void set_gyr_cov_scale(const Eigen::Vector3d& scaler);
  void set_acc_cov_scale(const Eigen::Vector3d& scaler);
  void set_gyr_bias_cov(const Eigen::Vector3d& b_g);
  void set_acc_bias_cov(const Eigen::Vector3d& b_a);
  void set_inv_expo_cov(const double& inv_expo);
  void set_imu_init_frame_num(const int& num);
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  void Process2(LidarMeasureGroup& lidar_meas, StatesGroup& stat, PointCloudT::Ptr cur_pcl_un_);

  double IMU_mean_acc_norm;
  Eigen::Vector3d unbiased_gyr;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;
  Eigen::Vector3d cov_bias_gyr;
  Eigen::Vector3d cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time;
  bool imu_time_init = false;
  bool imu_need_init = true;
  Eigen::Matrix3d Eye3d;
  Eigen::Vector3d Zero3d;

private:
  void IMU_init(const MeasureGroup& meas, StatesGroup& state, int& N);
  void UndistortPcl(LidarMeasureGroup& lidar_meas, StatesGroup& state_inout, PointCloudT& pcl_out);

  PointCloudT pcl_wait_proc;
  ImuData::ConstPtr last_imu;
  std::vector<Pose6D> IMUpose;
  Eigen::Matrix3d Lid_rot_to_IMU;
  Eigen::Vector3d Lid_offset_to_IMU;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;
  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;
  double last_prop_end_time;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace fastlivo
}  // namespace asuka
