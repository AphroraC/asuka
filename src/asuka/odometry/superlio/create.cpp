#include <asuka/odometry/superlio/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::superlio::OdometryEstimation();
}
