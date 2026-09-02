#include <asuka/odometry/smallpointlio/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::smallpointlio::OdometryEstimation();
}
