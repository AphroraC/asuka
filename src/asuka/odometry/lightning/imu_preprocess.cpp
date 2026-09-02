#include <asuka/odometry/lightning/imu_preprocess.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace lightning {

ImuPreprocess::ImuPreprocess() {
  logger = create_module_logger("preprocess");
  pimu = std::make_shared<::lightning::ImuProcess>();

  const Config sensor_config(GlobalConfig::get_config_path("config_sensor"));
  std::string lidar_key;
  if (sensor_config.has("livox"))
    lidar_key = "livox";
  else if (sensor_config.has("robosense"))
    lidar_key = "robosense";
  else
    throw std::runtime_error("config_sensor: no active lidar block");
  std::vector<std::string> nest{lidar_key};

  const Eigen::Vector3d extrinsic_t =
    sensor_config.param_nested<Eigen::Vector3d>(nest, "extrinsic_T", Eigen::Vector3d::Zero());
  const Eigen::Matrix3d extrinsic_r =
    sensor_config.param_nested<Eigen::Matrix3d>(nest, "extrinsic_R", Eigen::Matrix3d::Identity());
  pimu->SetExtrinsic(extrinsic_t, extrinsic_r);

  const double gyr_cov = sensor_config.param_nested<double>(nest, "gyr_cov", 0.1);
  const double acc_cov = sensor_config.param_nested<double>(nest, "acc_cov", 0.1);
  const double b_gyr_cov = sensor_config.param_nested<double>(nest, "b_gyr_cov", 0.0001);
  const double b_acc_cov = sensor_config.param_nested<double>(nest, "b_acc_cov", 0.0001);
  pimu->SetGyrCov(::lightning::Vec3d(gyr_cov, gyr_cov, gyr_cov));
  pimu->SetAccCov(::lightning::Vec3d(acc_cov, acc_cov, acc_cov));
  pimu->SetGyrBiasCov(::lightning::Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  pimu->SetAccBiasCov(::lightning::Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  const bool use_imu_filter = odometry_config.param<bool>("odometry", "imu_filter", false);
  pimu->SetUseIMUFilter(use_imu_filter);
}

void ImuPreprocess::initialize(double lidar_start_time) {
  (void)lidar_start_time;
}

void ImuPreprocess::process(const ImuMeasureGroup& meas, ::lightning::ESKF& kf_state, PointCloudT::Ptr& pcl_out) {
  ::lightning::MeasureGroup lmeas;
  lmeas.timestamp_ = meas.lidar_beg_time;
  lmeas.lidar_begin_time_ = meas.lidar_beg_time;
  lmeas.lidar_end_time_ = meas.lidar_end_time;
  lmeas.scan_ = meas.lidar;
  lmeas.scan_raw_ = meas.lidar;

  for (const auto& imu : meas.imu) {
    auto limu = std::make_shared<::lightning::IMU>();
    limu->timestamp = imu->stamp;
    limu->angular_velocity = imu->angular_vel;
    limu->linear_acceleration = imu->linear_acc;
    lmeas.imu_.push_back(limu);
  }

  pimu->Process(lmeas, kf_state, pcl_out);
}

bool ImuPreprocess::is_initialized() const {
  return pimu->IsIMUInited();
}

Eigen::Vector3d ImuPreprocess::get_mean_acceleration() const {
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d ImuPreprocess::get_mean_gyroscope() const {
  return Eigen::Vector3d::Zero();
}

}  // namespace lightning
}  // namespace asuka
