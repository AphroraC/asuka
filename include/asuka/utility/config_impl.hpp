#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <nlohmann/json.hpp>

#include <spdlog/spdlog.h>

#include <asuka/utility/config.hpp>

namespace asuka {

namespace {

template <typename T>
struct ConfigTraits {
  using InType = T;
  using OutType = T;
  static std::optional<OutType> convert(const InType& in) { return in; }
  static InType invert(const OutType& value) { return value; }
};

template <typename T, int N, int M>
struct ConfigTraits<Eigen::Matrix<T, N, M>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Matrix<T, N, M>;
  static std::optional<OutType> convert(const InType& in) {
    if (static_cast<int>(in.size()) != N * M) return std::nullopt;
    if constexpr (M == 1) {
      return Eigen::Map<const OutType>(in.data());
    } else {
      Eigen::Matrix<T, N, M, Eigen::RowMajor> row_major =
          Eigen::Map<const Eigen::Matrix<T, N, M, Eigen::RowMajor>>(in.data());
      return OutType(row_major);
    }
  }
  static InType invert(const OutType& value) {
    if constexpr (M == 1) {
      return std::vector<double>(value.data(), value.data() + N * M);
    } else {
      Eigen::Matrix<T, N, M, Eigen::RowMajor> row_major(value);
      return std::vector<double>(row_major.data(), row_major.data() + N * M);
    }
  }
};

template <typename T>
struct ConfigTraits<Eigen::Quaternion<T>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Quaternion<T>;
  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 4) return std::nullopt;
    return OutType(in.data()).normalized();
  }
  static InType invert(const OutType& value) { return std::vector<double>{value.x(), value.y(), value.z(), value.w()}; }
};

template <typename T>
struct ConfigTraits<Eigen::Transform<T, 3, Eigen::Isometry>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Transform<T, 3, Eigen::Isometry>;
  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 7) return std::nullopt;
    OutType se3 = OutType::Identity();
    se3.translation() = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(in.data());
    se3.linear() = Eigen::Quaternion<T>(in.data() + 3).normalized().toRotationMatrix();
    return se3;
  }
  static InType invert(const OutType& value) {
    Eigen::Matrix<T, 3, 1> t = value.translation();
    Eigen::Quaternion<T> q(value.linear());
    return std::vector<double>{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  }
};

}  // namespace

template <typename T>
std::optional<T> Config::param(const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  auto module = json.find(module_name);
  if (module == json.end()) return std::nullopt;
  auto parameter = module->find(param_name);
  if (parameter == module->end()) return std::nullopt;
  return ConfigTraits<T>::convert(parameter->get<typename ConfigTraits<T>::InType>());
}

template <typename T>
T Config::param(const std::string& module_name, const std::string& param_name, const T& default_value) const {
  auto found = param<T>(module_name, param_name);
  if (!found) {
    spdlog::warn("param {}/{} not found, using default", module_name, param_name);
    return default_value;
  }
  return *found;
}

template <typename T>
T Config::param_cast(const std::string& module_name, const std::string& param_name) const {
  auto found = param<T>(module_name, param_name);
  if (!found) {
    spdlog::critical("param {}/{} not found", module_name, param_name);
    std::abort();
  }
  return *found;
}

template <typename T>
std::optional<T> Config::param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name)
  const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  if (nested_module_names.empty()) return std::nullopt;

  nlohmann::json::const_iterator itr = json.find(nested_module_names[0]);
  if (itr == json.end()) return std::nullopt;
  for (size_t i = 1; i < nested_module_names.size(); ++i) {
    auto next = itr->find(nested_module_names[i]);
    if (next == itr->end()) return std::nullopt;
    itr = next;
  }
  auto parameter = itr->find(param_name);
  if (parameter == itr->end()) return std::nullopt;
  return ConfigTraits<T>::convert(parameter->get<typename ConfigTraits<T>::InType>());
}

template <typename T>
T Config::param_nested(
  const std::vector<std::string>& nested_module_names,
  const std::string& param_name,
  const T& default_value) const {
  auto found = param_nested<T>(nested_module_names, param_name);
  if (!found) {
    std::stringstream ss;
    for (const auto& m : nested_module_names) ss << m << "/";
    spdlog::warn("param {}{} not found, using default", ss.str(), param_name);
    return default_value;
  }
  return *found;
}

template <typename T>
T Config::param_cast_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const {
  auto found = param_nested<T>(nested_module_names, param_name);
  if (!found) {
    std::stringstream ss;
    for (const auto& m : nested_module_names) ss << m << "/";
    spdlog::critical("param {}{} not found", ss.str(), param_name);
    std::abort();
  }
  return *found;
}

template <typename T>
bool Config::override_param(const std::string& module_name, const std::string& param_name, const T& value) {
  auto& json = std::any_cast<nlohmann::json&>(config);
  json[module_name][param_name] = ConfigTraits<T>::invert(value);
  return true;
}

#define ASUKA_DEFINE_CONFIG_IO(TYPE)                                                                                   \
  template std::optional<TYPE> Config::param(const std::string&, const std::string&) const;                          \
  template TYPE Config::param(const std::string&, const std::string&, const TYPE&) const;                            \
  template TYPE Config::param_cast(const std::string&, const std::string&) const;                                     \
  template std::optional<TYPE> Config::param_nested(const std::vector<std::string>&, const std::string&) const;       \
  template TYPE Config::param_nested(const std::vector<std::string>&, const std::string&, const TYPE&) const;         \
  template TYPE Config::param_cast_nested(const std::vector<std::string>&, const std::string&) const;                  \
  template bool Config::override_param(const std::string&, const std::string&, const TYPE&);

}  // namespace asuka
