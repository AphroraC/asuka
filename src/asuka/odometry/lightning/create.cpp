#include <asuka/odometry/lightning/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::lightning::OdometryEstimation();
}
