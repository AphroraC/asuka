#include <asuka/odometry/smallpointlio/imu_preprocess.hpp>

namespace asuka {
namespace smallpointlio {

ImuPreprocess::ImuPreprocess() {
  logger = create_module_logger("preprocess");
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
