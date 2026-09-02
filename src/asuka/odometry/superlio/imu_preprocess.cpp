#include "asuka/odometry/superlio/imu_preprocess.hpp"

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace superlio {

ImuPreprocess::ImuPreprocess() {
  logger = create_module_logger("preprocess");

  const Config config(GlobalConfig::get_config_path("config_odometry"));
  imu_buffer.set_capacity(config.param<int>("odometry", "imu_buffer_capacity", 1000));
}

void ImuPreprocess::insert_imu(const ImuData::ConstPtr& imu) {
  ImuSample sample;
  sample.secs = imu->stamp;
  sample.acc = imu->linear_acc.cast<Scalar>();
  sample.gyr = imu->angular_vel.cast<Scalar>();

  if (sample.secs < last_timestamp) {
    logger->warn("imu loop back, clear buffer: {} -> {}", last_timestamp, sample.secs);
    imu_buffer.clear();
  }
  imu_buffer.push_back(sample);
  last_timestamp = sample.secs;
}

void ImuPreprocess::accumulate(const ImuSample& imu) {
  imu_count++;
  mean_gyro += (imu.gyr - mean_gyro) / static_cast<Scalar>(imu_count);
  mean_acc += (imu.acc - mean_acc) / static_cast<Scalar>(imu_count);
}

void ImuPreprocess::reset_accumulators() {
  imu_count = 0;
  mean_gyro = V3::Zero();
  mean_acc = V3::Zero();
}

}  // namespace superlio
}  // namespace asuka
