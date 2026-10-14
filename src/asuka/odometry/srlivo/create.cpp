#include <asuka/odometry/srlivo/odometry_estimation.hpp>

extern "C" asuka::OdometryEstimation *create_odometry_estimation() {
  return new asuka::srlivo::OdometryEstimation();
}
