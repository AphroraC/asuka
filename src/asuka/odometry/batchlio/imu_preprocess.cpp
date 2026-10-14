#include <asuka/odometry/batchlio/imu_preprocess.hpp>

namespace asuka {
namespace batchlio {

ImuPreprocess::ImuPreprocess() : b_first_frame(true), imu_need_init(true) {
  imu_enabled = true;
  init_iter_num = 1;
  mean_acc = Eigen::Vector3d(0, 0, 0.0);
  mean_gyro = Eigen::Vector3d(0, 0, 0);
  after_imu_init = false;
  state_cov.setIdentity();
  logger = create_module_logger("odometry");
}

void ImuPreprocess::reset() {
  mean_acc = Eigen::Vector3d(0, 0, 0.0);
  mean_gyro = Eigen::Vector3d(0, 0, 0);
  imu_need_init = true;
  init_iter_num = 1;
  after_imu_init = false;
  time_last_scan = 0.0;
}

void ImuPreprocess::initialize(double lidar_start_time) {
  (void)lidar_start_time;
  reset();
}

void ImuPreprocess::set_init(Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot) {
  Eigen::Matrix3d hat_grav;
  hat_grav << 0.0, gravity_value(2), -gravity_value(1), -gravity_value(2), 0.0, gravity_value(0), gravity_value(1),
    -gravity_value(0), 0.0;
  const double align_norm = (hat_grav * tmp_gravity).norm() / gravity_value.norm() / tmp_gravity.norm();
  double align_cos = gravity_value.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_value.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6) {
    if (align_cos > 1e-6) {
      rot = Eigen::Matrix3d::Identity();
    } else {
      rot = -Eigen::Matrix3d::Identity();
    }
  } else {
    Eigen::Vector3d align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * std::acos(align_cos);
    rot = batch::exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

void ImuPreprocess::imu_init(const batch::MeasureGroup& meas, int& n) {
  Eigen::Vector3d cur_acc;
  Eigen::Vector3d cur_gyr;
  if (b_first_frame) {
    reset();
    n = 1;
    b_first_frame = false;
    const auto& imu_acc = meas.imu.front()->linear_acc;
    const auto& gyr_acc = meas.imu.front()->angular_vel;
    mean_acc << imu_acc(0), imu_acc(1), imu_acc(2);
    mean_gyro << gyr_acc(0), gyr_acc(1), gyr_acc(2);
  }
  for (const auto& imu : meas.imu) {
    const auto& imu_acc = imu->linear_acc;
    const auto& gyr_acc = imu->angular_vel;
    cur_acc << imu_acc(0), imu_acc(1), imu_acc(2);
    cur_gyr << gyr_acc(0), gyr_acc(1), gyr_acc(2);
    mean_acc += (cur_acc - mean_acc) / n;
    mean_gyro += (cur_gyr - mean_gyro) / n;
    n++;
  }
}

void ImuPreprocess::process(const batch::MeasureGroup& meas, PointCloudT::Ptr& pcl_out) {
  if (imu_enabled) {
    const bool has_initialization_imu = !meas.imu_initialization.empty();
    if (meas.imu.empty() && !has_initialization_imu) return;
    if (imu_need_init) {
      if (has_initialization_imu) {
        batch::MeasureGroup initialization_measurements = meas;
        initialization_measurements.imu = meas.imu_initialization;
        imu_init(initialization_measurements, init_iter_num);
      } else {
        imu_init(meas, init_iter_num);
      }
      imu_need_init = true;
      if (init_iter_num > MAX_INI_COUNT) {
        imu_need_init = false;
        *pcl_out = *(meas.lidar);
      }
      return;
    }
    if (!after_imu_init) after_imu_init = true;
    *pcl_out = *(meas.lidar);
    return;
  }
  *pcl_out = *(meas.lidar);
}

}  // namespace batchlio
}  // namespace asuka
