#include <asuka/core/odometry_estimation.hpp>

#include <stdexcept>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>
#include <asuka/utility/load_module.hpp>

namespace asuka {

std::shared_ptr<OdometryEstimation> OdometryEstimation::load_module(const std::string& so_name) {
  return load_module_from_so<OdometryEstimation>(so_name, "create_odometry_estimation");
}

std::shared_ptr<OdometryEstimation> OdometryEstimation::load_configured() {
  const Config odometry_config = load_config("config_odometry");
  const std::string so_name = odometry_config.param<std::string>("odometry", "so_name", "");
  auto odometry = load_module(so_name);
  if (!odometry) {
    throw std::runtime_error("failed to load odometry module: " + so_name);
  }
  return odometry;
}

}  // namespace asuka
