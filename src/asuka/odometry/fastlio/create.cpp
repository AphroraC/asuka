#include <asuka/odometry/fastlio/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::fastlio::OdometryEstimation();
}
