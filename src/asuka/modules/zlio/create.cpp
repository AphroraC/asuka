#include <asuka/modules/zlio/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::zlio::OdometryEstimation();
}
