
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <spdlog/spdlog.h>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

Config::Config(const std::string& config_filename) {
  nlohmann::json json;
  if (config_filename.empty()) {
    config = json;
    return;
  }

  // A config file that cannot be opened or parsed is a hard error: falling
  // back to an empty json would silently run every parameter at its default
  // value -- an untraceable trajectory-level misconfiguration.
  std::ifstream ifs(config_filename);
  if (!ifs) {
    throw std::runtime_error("failed to open config file: " + config_filename);
  }
  try {
    json = nlohmann::json::parse(ifs, nullptr, true, true);
  } catch (const nlohmann::json::exception& e) {
    throw std::runtime_error("failed to parse config file " + config_filename + ": " + e.what());
  }
  config = json;
}

Config::~Config() {}

bool Config::has_param(const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  auto module = json.find(module_name);
  if (module == json.end()) return false;
  return module->find(param_name) != module->end();
}

bool Config::has(const std::string& module_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  return json.find(module_name) != json.end();
}

void Config::save(const std::string& path) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  std::ofstream ofs(path);
  ofs << std::setw(2) << json << std::endl;
}

ASUKA_DEFINE_CONFIG_IO(bool)
ASUKA_DEFINE_CONFIG_IO(int)
ASUKA_DEFINE_CONFIG_IO(size_t)
ASUKA_DEFINE_CONFIG_IO(float)
ASUKA_DEFINE_CONFIG_IO(double)
ASUKA_DEFINE_CONFIG_IO(std::string)
ASUKA_DEFINE_CONFIG_IO(std::vector<int>)
ASUKA_DEFINE_CONFIG_IO(std::vector<double>)
ASUKA_DEFINE_CONFIG_IO(std::vector<std::string>)
ASUKA_DEFINE_CONFIG_IO(Eigen::Vector3d)
ASUKA_DEFINE_CONFIG_IO(Eigen::Vector4d)
ASUKA_DEFINE_CONFIG_IO(Eigen::Matrix3d)
ASUKA_DEFINE_CONFIG_IO(Eigen::Quaterniond)
ASUKA_DEFINE_CONFIG_IO(Eigen::Isometry3d)

GlobalConfig::GlobalConfig(const std::string& global_config_path, const std::string& config_dir)
: Config(global_config_path) {
  override_param("global", "config_path", config_dir);
}

GlobalConfig* GlobalConfig::instance(const std::string& config_path) {
  // Thread-safe one-time init; the directory is fixed at the first call.
  // Modules resolve their config files at construction, so re-pointing the
  // directory mid-process never worked anyway -- and the old
  // delete-and-recreate was neither thread-safe nor double-instance safe.
  // configure_logging() runs after the instance is fully constructed (it
  // reads logging params back through instance(); calling it inside the
  // constructor would re-enter the not-yet-initialized static -- undefined).
  static GlobalConfig inst(config_path + "/config.json", config_path);
  // No capture: `inst` has static storage duration and is already
  // initialized here; capturing it would only trigger -Wcapture warnings.
  static const bool logged = [] {
    configure_logging(inst);
    return true;
  }();
  (void)logged;
  return &inst;
}

std::string GlobalConfig::get_config_path(const std::string& config_name) {
  auto global = instance();
  const std::string directory = global->param<std::string>("global", "config_path", ".");
  const std::string filename = global->param<std::string>("global", config_name, config_name + ".json");
  return directory + "/" + filename;
}

void GlobalConfig::dump(const std::string& path) {
  spdlog::debug("dumping config to {}", path);
  std::filesystem::create_directories(path);
  this->save(path + "/config.json");
}

}  // namespace asuka
