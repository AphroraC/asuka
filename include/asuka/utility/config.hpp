#pragma once

#include <any>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace asuka {

class Config {
public:
  Config(const std::string& config_filename = std::string());
  virtual ~Config();

  bool has_param(const std::string& module_name, const std::string& param_name) const;

  bool has(const std::string& module_name) const;

  template <typename T>
  std::optional<T> param(const std::string& module_name, const std::string& param_name) const;

  template <typename T>
  T param(const std::string& module_name, const std::string& param_name, const T& default_value) const;

  template <typename T>
  T param_cast(const std::string& module_name, const std::string& param_name) const;

  template <typename T>
  std::optional<T> param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name)
    const;

  template <typename T>
  T param_nested(
    const std::vector<std::string>& nested_module_names,
    const std::string& param_name,
    const T& default_value) const;

  template <typename T>
  T param_cast_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const;

  template <typename T>
  bool override_param(const std::string& module_name, const std::string& param_name, const T& value);

  void save(const std::string& path) const;

  friend class GlobalConfig;

protected:
  std::any config;
};

class GlobalConfig : public Config {
private:
  // Sets up the path routing and logging exactly once, at first-call time.
  GlobalConfig(const std::string& global_config_path, const std::string& config_dir);

public:
  // Function-local static singleton: fixed at the first call, later calls
  // reuse the instance even with a different config_path.
  static GlobalConfig* instance(const std::string& config_path = std::string());
  static std::string get_config_path(const std::string& config_name);
  void dump(const std::string& path);
};

/// Convenience loader for a routed config name, e.g. load_config("config_ros").
/// Returns the Config by value (guaranteed copy elision -- no json copy).
inline Config load_config(const std::string& name) {
  return Config(GlobalConfig::get_config_path(name));
}

}  // namespace asuka
