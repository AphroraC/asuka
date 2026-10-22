/**
 * @file config.cpp
 * @brief 读取并校验激光惯性里程计的算法参数。
 * 读取层为 asuka 配置系统（config_zlio.json，键结构与上游 lio.yaml 同构），
 * 校验与归一化层逐字保留自上游实现。
 */
#include <asuka/odometry/zlio/config.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace zlio {
namespace {

using ModulePath = std::vector<std::string>;

/// 读取存在的标量配置并保留缺省值（缺键回退当前值并记录 warning）。
template <typename T>
void read(const asuka::Config& config, const ModulePath& module, const char* key, T& value) {
  value = config.param_nested<T>(module, key, value);
}

/// 读取固定长度的数值配置数组。
template <std::size_t N>
void read(const asuka::Config& config, const ModulePath& module, const char* key,
          std::array<double, N>& value) {
  const std::vector<double> current(value.begin(), value.end());
  const std::vector<double> values = config.param_nested<std::vector<double>>(module, key, current);
  if (values.size() != N)
    throw std::invalid_argument(std::string(key) + " has an invalid array length");
  std::copy(values.begin(), values.end(), value.begin());
}

/// 读取惯性采样方差及两类偏置随机游走强度。
void readImuNoise(const asuka::Config& config, const ModulePath& module, ImuNoise& noise) {
  read(config, module, "gyro_variance", noise.gyro_sample_variance);
  read(config, module, "accel_variance", noise.accel_sample_variance);
  read(config, module, "gyro_bias_psd", noise.gyro_bias_psd);
  read(config, module, "accel_bias_psd", noise.accel_bias_psd);
}

/// 校验数值为有限正数。
void positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
}

/// 按配置路径校验惯性测量和偏置噪声。
void validateImuNoise(const ImuNoise& noise, const std::string& path) {
  positive(noise.gyro_sample_variance, (path + ".gyro_variance").c_str());
  positive(noise.accel_sample_variance, (path + ".accel_variance").c_str());
  positive(noise.gyro_bias_psd, (path + ".gyro_bias_psd").c_str());
  positive(noise.accel_bias_psd, (path + ".accel_bias_psd").c_str());
}

/// 读取重力、测距范围和传感器外参。
void readSensor(const asuka::Config& config, Config& out) {
  const ModulePath module{"sensor"};
  read(config, module, "gravity", out.gravity);
  read(config, module, "minimum_range", out.minimum_range);
  read(config, module, "maximum_range", out.maximum_range);
  const ModulePath extrinsic{"sensor", "body_from_lidar"};
  std::array<double, 3> position{0, 0, 0};
  std::array<double, 4> rotation{0, 0, 0, 1};
  read(config, extrinsic, "translation", position);
  read(config, extrinsic, "rotation_xyzw", rotation);
  out.body_from_lidar.position = Eigen::Map<Vec3>(position.data());
  out.body_from_lidar.rotation =
    Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2]);
}

/// 读取五组导航误差方差，每组应用于对应的三个轴。
void readUncertainty(const asuka::Config& config, const ModulePath& module, InitialUncertainty& prior) {
  read(config, module, "allow_configured_fallback", prior.allow_configured_fallback);
  std::array<double, 5> variance{prior.variance[0][0], prior.variance[1][0], prior.variance[2][0],
                                 prior.variance[3][0], prior.variance[4][0]};
  read(config, module, "state_variance", variance);
  for (int i = 0; i < 5; ++i)
    prior.variance[i].setConstant(variance[i]);
}

/// 读取启动粗配准参数。
void readRegistration(const asuka::Config& config, const ModulePath& module, RegistrationConfig& out) {
  read(config, module, "voxel_size", out.voxel_size);
  read(config, module, "neighbors", out.neighbors);
  read(config, module, "minimum_map_points", out.minimum_map_points);
  read(config, module, "maximum_iterations", out.maximum_iterations);
  read(config, module, "plane_residual_tolerance", out.plane_residual_tolerance);
  read(config, module, "prior_covariance_scale", out.prior_covariance_scale);
  read(config, module, "rotation_tolerance", out.rotation_tolerance);
  read(config, module, "position_tolerance", out.position_tolerance);
  read(config, module, "gyro_variance", out.gyro_variance);
  read(config, module, "accel_variance", out.accel_variance);
  read(config, module, "bias_variance", out.bias_variance);
  read(config, module, "initial_state_variance", out.initial_state_variance);
  read(config, module, "initial_bias_variance", out.initial_bias_variance);
}

/// 读取启动窗口优化参数。
void readStartupOptimization(const asuka::Config& config, const ModulePath& module,
                             StartupOptimizationConfig& out) {
  read(config, module, "voxel_size", out.voxel_size);
  read(config, module, "maximum_depth", out.maximum_depth);
  read(config, module, "minimum_points", out.minimum_points);
  read(config, module, "minimum_planes", out.minimum_planes);
  read(config, module, "plane_variance", out.plane_variance);
  read(config, module, "plane_eigenvalue_ratio", out.plane_eigenvalue_ratio);
  read(config, module, "plane_middle_ratio", out.plane_middle_ratio);
  read(config, module, "imu_weight", out.imu_weight);
  read(config, module, "thread_count", out.thread_count);
  read(config, module, "maximum_iterations", out.maximum_iterations);
  readImuNoise(config, module, out.noise);
}

/// 读取启动窗口、偏置和求解设置。
void readInitialization(const asuka::Config& config, Config& out) {
  const ModulePath module{"initialization"};
  read(config, module, "window_frames", out.initialization.window_frames);
  read(config, module, "seed_imu_samples", out.initialization.seed_imu_samples);
  read(config, module, "maximum_refinements", out.initialization.maximum_refinements);
  read(config, module, "sample_size", out.initialization.sample_size);
  read(config, module, "fallback_sample_size", out.initialization.fallback_sample_size);
  read(config, module, "minimum_sampled_points", out.initialization.minimum_sampled_points);
  read(config, module, "minimum_directional_support", out.initialization.minimum_directional_support);
  read(config, module, "relative_cost_tolerance", out.initialization.relative_cost_tolerance);
  readRegistration(config, {"initialization", "registration"}, out.initialization.registration);
  readStartupOptimization(config, {"initialization", "optimization"}, out.initialization.optimization);
  std::array<double, 3> bias{0, 0, 0};
  read(config, module, "accel_bias", bias);
  out.initial_accel_bias = Eigen::Map<Vec3>(bias.data());
  std::array<double, 3> deviations;
  std::copy_n(out.initial_accel_bias_std.data(), 3, deviations.begin());
  read(config, module, "accel_bias_std", deviations);
  out.initial_accel_bias_std = Eigen::Map<Vec3>(deviations.data());
  readUncertainty(config, {"initialization", "uncertainty"}, out.initial_uncertainty);
}

/// 读取运行滤波参数。
void readFilter(const asuka::Config& config, Config& out) {
  const ModulePath module{"filter"};
  read(config, module, "maximum_iterations", out.maximum_iterations);
  read(config, module, "association_threads", out.association_threads);
  read(config, module, "convergence_tolerance", out.convergence_tolerance);
  readImuNoise(config, module, out.noise);
}

/// 读取概率地图参数。
void readMap(const asuka::Config& config, const ModulePath& module, MapConfig& map) {
  read(config, module, "voxel_size", map.voxel_size);
  read(config, module, "range_sigma", map.range_sigma);
  read(config, module, "angle_sigma_degrees", map.angle_sigma_degrees);
  read(config, module, "maximum_roots", map.maximum_roots);
}

/// 读取自适应采样参数。
void readSampling(const asuka::Config& config, const ModulePath& module, SamplingConfig& sampling) {
  read(config, module, "initial_size", sampling.initial_size);
  read(config, module, "minimum_size", sampling.minimum_size);
  read(config, module, "maximum_size", sampling.maximum_size);
  read(config, module, "history_size", sampling.history_size);
  read(config, module, "minimum_points", sampling.minimum_points);
  read(config, module, "maximum_points", sampling.maximum_points);
  read(config, module, "scale_threshold", sampling.scale_threshold);
  read(config, module, "exponent", sampling.exponent);
  read(config, module, "proportional_scale", sampling.proportional_scale);
  read(config, module, "derivative_scale", sampling.derivative_scale);
  read(config, module, "proportional_minimum", sampling.proportional_minimum);
  read(config, module, "proportional_maximum", sampling.proportional_maximum);
  read(config, module, "derivative_minimum", sampling.derivative_minimum);
  read(config, module, "derivative_maximum", sampling.derivative_maximum);
}

/// 读取连续时间调度的间隔边界和归一化局部误差预算。
void readWindow(const asuka::Config& config, const ModulePath& module, WindowConfig& window) {
  read(config, module, "maximum_interval", window.maximum_interval);
  read(config, module, "minimum_interval", window.minimum_interval);
  read(config, module, "normalized_error", window.normalized_error);
}

/// 校验采样尺度、目标数量和反馈增益。
void validateSampling(const SamplingConfig& s) {
  positive(s.minimum_size, "sampling.minimum_size");
  positive(s.maximum_size, "sampling.maximum_size");
  positive(s.initial_size, "sampling.initial_size");
  positive(s.minimum_points, "sampling.minimum_points");
  positive(s.maximum_points, "sampling.maximum_points");
  positive(s.scale_threshold, "sampling.scale_threshold");
  positive(s.proportional_scale, "sampling.proportional_scale");
  positive(s.derivative_scale, "sampling.derivative_scale");
  positive(s.proportional_minimum, "sampling.proportional_minimum");
  positive(s.proportional_maximum, "sampling.proportional_maximum");
  positive(s.derivative_minimum, "sampling.derivative_minimum");
  positive(s.derivative_maximum, "sampling.derivative_maximum");
  if (s.history_size == 0 || s.minimum_size > s.initial_size || s.initial_size > s.maximum_size ||
      s.minimum_points > s.maximum_points || !std::isfinite(s.exponent) || s.exponent <= 1 ||
      s.proportional_minimum > s.proportional_maximum ||
      s.derivative_minimum > s.derivative_maximum)
    throw std::invalid_argument("invalid sampling parameter range");
}

/// 校验启动窗口和粗配准设置。
void validateInitialization(const InitializationConfig& config) {
  if (config.window_frames < 2 || config.window_frames > std::numeric_limits<int>::max() / 15 ||
      config.seed_imu_samples == 0 || config.maximum_refinements < 4 ||
      config.minimum_sampled_points == 0)
    throw std::invalid_argument("invalid initialization window or iteration budget");
  positive(config.sample_size, "initialization.sample_size");
  positive(config.fallback_sample_size, "initialization.fallback_sample_size");
  positive(config.minimum_directional_support, "initialization.minimum_directional_support");
  if (config.fallback_sample_size > config.sample_size)
    throw std::invalid_argument(
      "initialization fine sampling size must not exceed the regular size");
  for (double tolerance : config.relative_cost_tolerance) {
    positive(tolerance, "initialization.relative_cost_tolerance");
    if (tolerance >= 1)
      throw std::invalid_argument(
        "initialization relative cost tolerance must be less than one");
  }
  const auto& registration = config.registration;
  positive(registration.voxel_size, "initialization.registration.voxel_size");
  positive(registration.plane_residual_tolerance,
           "initialization.registration.plane_residual_tolerance");
  positive(registration.prior_covariance_scale,
           "initialization.registration.prior_covariance_scale");
  positive(registration.rotation_tolerance, "initialization.registration.rotation_tolerance");
  positive(registration.position_tolerance, "initialization.registration.position_tolerance");
  positive(registration.gyro_variance, "initialization.registration.gyro_variance");
  positive(registration.accel_variance, "initialization.registration.accel_variance");
  positive(registration.bias_variance, "initialization.registration.bias_variance");
  positive(registration.initial_state_variance,
           "initialization.registration.initial_state_variance");
  positive(registration.initial_bias_variance,
           "initialization.registration.initial_bias_variance");
  if (registration.neighbors < 3 || registration.maximum_iterations < 1 ||
      registration.minimum_map_points < static_cast<std::size_t>(registration.neighbors))
    throw std::invalid_argument(
      "invalid coarse initialization registration point count or iteration budget");
}

/// 校验启动几何与优化资源设置。
void validateStartupOptimization(const StartupOptimizationConfig& config) {
  positive(config.voxel_size, "initialization.optimization.voxel_size");
  positive(config.imu_weight, "initialization.optimization.imu_weight");
  positive(config.plane_variance[0], "initialization.optimization.plane_variance[0]");
  positive(config.plane_variance[1], "initialization.optimization.plane_variance[1]");
  positive(config.plane_eigenvalue_ratio, "initialization.optimization.plane_eigenvalue_ratio");
  positive(config.plane_middle_ratio, "initialization.optimization.plane_middle_ratio");
  validateImuNoise(config.noise, "initialization.optimization");
  if (config.maximum_depth < 0 || config.maximum_depth > 8 || config.thread_count < 1 ||
      config.maximum_iterations < 1 || config.minimum_points < 3 || config.minimum_planes < 1 ||
      config.plane_variance[1] > config.plane_variance[0] || config.plane_eigenvalue_ratio >= 1 ||
      config.plane_middle_ratio >= 1)
    throw std::invalid_argument("invalid initialization plane or solver resource parameters");
}

/// 校验观测间隔的数值下限及上下界关系。
void validateWindow(const WindowConfig& window) {
  positive(window.maximum_interval, "window.maximum_interval");
  positive(window.minimum_interval, "window.minimum_interval");
  positive(window.normalized_error, "window.normalized_error");
  if (window.minimum_interval < 1e-9 || 2 * window.minimum_interval > window.maximum_interval)
    throw std::invalid_argument(
      "intervals must satisfy 1 ns <= minimum_interval and 2*minimum_interval <= "
      "maximum_interval");
}
/// 校验传感器几何、重力和启动先验。
void validateInitialConditions(const Config& config) {
  if (!std::isfinite(config.gravity) || config.gravity < 9.6 || config.gravity > 10.0)
    throw std::invalid_argument("sensor.gravity must be within [9.6, 10.0] m/s^2");
  if (!config.initial_accel_bias.allFinite())
    throw std::invalid_argument("invalid initial acceleration bias");
  for (int axis = 0; axis < 3; ++axis) {
    const double deviation = config.initial_accel_bias_std[axis];
    positive(deviation, "initialization.accel_bias_std");
    positive(1 / (deviation * deviation), "acceleration bias prior information");
  }
  positive(config.maximum_range, "sensor.maximum_range");
  if (!std::isfinite(config.minimum_range) || config.minimum_range < 0 ||
      config.minimum_range >= config.maximum_range)
    throw std::invalid_argument("invalid range limits");
  if (!config.body_from_lidar.position.allFinite() ||
      !config.body_from_lidar.rotation.coeffs().allFinite() ||
      std::abs(config.body_from_lidar.rotation.norm() - 1) > .01)
    throw std::invalid_argument("invalid rigid extrinsics");
  for (const Vec3& variance : config.initial_uncertainty.variance)
    for (int axis = 0; axis < 3; ++axis)
      positive(variance[axis], "variance component in initialization.uncertainty");
}

/// 校验惯性噪声、线程数及迭代预算。
void validateFilter(const Config& config) {
  validateImuNoise(config.noise, "filter");
  if (config.maximum_iterations < 1)
    throw std::invalid_argument("filter iteration count must be positive");
  if (config.association_threads < 1 || config.association_threads > 2)
    throw std::invalid_argument("point-to-plane association thread count must be one or two");
  positive(config.convergence_tolerance, "filter.convergence_tolerance");
}

/// 校验地图平面判据、测量噪声和资源上限。
void validateMap(const MapConfig& map) {
  positive(map.voxel_size, "map.voxel_size");
  positive(map.range_sigma, "map.range_sigma");
  positive(map.angle_sigma_degrees, "map.angle_sigma_degrees");
  const float resolution = static_cast<float>(map.voxel_size);
  if (!std::isnormal(resolution) || !std::isnormal(1.f / resolution))
    throw std::invalid_argument("map scale must be representable as a valid float");
  if (map.maximum_roots == 0)
    throw std::invalid_argument("map root voxel capacity must be positive");
}
}  // namespace

/// 加载并规范化算法配置（config_zlio.json，键结构与上游 lio.yaml 同构）。
Config loadConfig(const asuka::Config& config) {
  Config out;
  readSensor(config, out);
  readInitialization(config, out);
  readFilter(config, out);
  readMap(config, {"map"}, out.map);
  readSampling(config, {"sampling"}, out.sampling);
  readWindow(config, {"window"}, out.window);
  return normalizeConfig(std::move(out));
}

/// 校验配置并规范化外参旋转。
Config normalizeConfig(Config config) {
  validateConfig(config);
  // 将允许范围内的外参四元数归一化。
  if (std::abs(config.body_from_lidar.rotation.squaredNorm() - 1) >
      4 * std::numeric_limits<double>::epsilon())
    config.body_from_lidar.rotation.normalize();
  return config;
}

/// 校验算法参数、坐标变换与资源设置。
void validateConfig(const Config& config) {
  validateInitialConditions(config);
  validateFilter(config);
  validateMap(config.map);
  validateSampling(config.sampling);
  validateWindow(config.window);
  validateInitialization(config.initialization);
  validateStartupOptimization(config.initialization.optimization);
}
}  // namespace zlio
}  // namespace asuka
