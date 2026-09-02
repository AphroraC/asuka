#include <asuka/odometry/smallpointlio/imu_preprocess.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace smallpointlio {

ImuPreprocess::ImuPreprocess() {
  logger = create_module_logger("preprocess");

  const Config config(GlobalConfig::get_config_path("config_odometry"));
  imu_deque.set_capacity(config.param<int>("odometry", "imu_buffer_capacity", 1000));
}

void ImuPreprocess::insert_imu(const ImuData::ConstPtr& imu) {
  if (imu->stamp < last_timestamp_imu) {
    logger->error("imu loop back detected");
    return;
  }
  imu_deque.push_back(imu);
  last_timestamp_imu = imu->stamp;
}

}  // namespace smallpointlio
}  // namespace asuka
