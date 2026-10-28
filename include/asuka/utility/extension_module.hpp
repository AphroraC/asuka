#pragma once

#include <memory>
#include <string>
#include <vector>

namespace asuka {

class ExtensionModule {
public:
  ExtensionModule() = default;
  virtual ~ExtensionModule() = default;

  virtual void at_exit(const std::string& dump_path) {}

  /// True while the module is healthy (worker thread running, no internal
  /// fault). AsukaROS::ok() aggregates this across the odometry executor and
  /// every loaded extension. Default: healthy; modules with failure modes
  /// override.
  virtual bool ok() const { return true; }

  virtual void stop() {}

  static std::shared_ptr<ExtensionModule> load_module(const std::string& so_name);

  static std::vector<std::shared_ptr<ExtensionModule>> load_configured_modules();
};

}  // namespace asuka
