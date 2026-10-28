#include <asuka/utility/extension_module.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>
#include <asuka/utility/load_module.hpp>

namespace asuka {

std::shared_ptr<ExtensionModule> ExtensionModule::load_module(const std::string& so_name) {
  return load_module_from_so<ExtensionModule>(so_name, "create_extension_module");
}

std::vector<std::shared_ptr<ExtensionModule>> ExtensionModule::load_configured_modules() {
  // config_extensions is optional: a missing or unparsable file degrades to
  // an empty module list with a warning -- the same policy as a module .so
  // that fails to load. Only the odometry config is a hard error.
  std::unique_ptr<Config> extensions_config;
  try {
    extensions_config = std::make_unique<Config>(GlobalConfig::get_config_path("config_extensions"));
  } catch (const std::exception& e) {
    spdlog::warn("config_extensions unavailable ({}); no extension modules loaded", e.what());
    return {};
  }
  const auto so_names = extensions_config->param<std::vector<std::string>>(
    "extensions",
    "loading_extension_modules",
    std::vector<std::string>());

  std::vector<std::shared_ptr<ExtensionModule>> modules;
  modules.reserve(so_names.size());
  for (const auto& so_name : so_names) {
    auto module = load_module(so_name);
    if (module) {
      modules.push_back(std::move(module));
    } else {
      spdlog::warn("failed to load extension module: {}", so_name);
    }
  }
  return modules;
}

}  // namespace asuka
