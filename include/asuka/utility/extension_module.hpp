#pragma once

#include <memory>
#include <string>

namespace asuka {

class ExtensionModule {
public:
  ExtensionModule() = default;
  virtual ~ExtensionModule() = default;

  virtual void at_exit(const std::string& dump_path) {}

  // Joins every thread the module owns and stops its slot emissions. Must be
  // idempotent and safe to call again from the destructor. AsukaROS calls
  // stop() on every loaded module before destroying any of them, so once
  // stop() has returned there are no in-flight slot emissions left and the
  // module can safely unsubscribe from the callback slots and destruct.
  virtual void stop() {}

  static std::shared_ptr<ExtensionModule> load_module(const std::string& so_name);
};

}  // namespace asuka
