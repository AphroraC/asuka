#include <asuka/modules/fastlivo/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
  return new asuka::fastlivo::OdometryEstimation();
}
