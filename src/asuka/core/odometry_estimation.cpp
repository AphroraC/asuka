#include <asuka/core/odometry_estimation.hpp>
#include <asuka/utility/load_module.hpp>

namespace asuka {

std::shared_ptr<OdometryEstimation> OdometryEstimation::load_module(const std::string& so_name) {
  return load_module_from_so<OdometryEstimation>(so_name, "create_odometry_estimation");
}

}  // namespace asuka
