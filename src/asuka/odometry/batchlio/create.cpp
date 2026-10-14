#include <asuka/odometry/batchlio/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::batchlio::OdometryEstimation();
}
