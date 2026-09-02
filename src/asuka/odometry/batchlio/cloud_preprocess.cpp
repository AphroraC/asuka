#include <algorithm>
#include <cmath>

#include <boost/make_shared.hpp>

#include <asuka/odometry/batchlio/cloud_preprocess.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace batchlio {
namespace {

bool finite_point(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

}  // namespace

CloudPreprocess::CloudPreprocess() : logger(create_module_logger("preprocess")) {
  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  const auto preprocess_value = [&](const std::string& key, double fallback) -> double {
    const auto found = odometry_config.param<double>("preprocess", key);
    return found ? *found : fallback;
  };
  min_distance = preprocess_value("min_distance", 0.5);
  max_distance = std::max(0.0, preprocess_value("max_distance", 300.0));
  point_filter_num = std::max(1, static_cast<int>(preprocess_value("point_filter_num", 2)));
  num_scans = std::max(1, static_cast<int>(preprocess_value("scan_line", 6)));
  scan_rate = std::max(1, static_cast<int>(preprocess_value("scan_rate", 10)));
}

PointCloudT::Ptr CloudPreprocess::build_livox_surf(const pcl::PointCloud<LivoxPoint>& raw) const {
  auto surf = boost::make_shared<PointCloudT>();
  const std::size_t size = raw.size();
  if (size == 0) return surf;
  surf->reserve(size);
  const double time_base = raw.front().timestamp;
  PointCloudT full;
  full.resize(size);
  unsigned int valid_count = 0;
  for (std::size_t i = 1; i < size; ++i) {
    const LivoxPoint& value = raw[i];
    if (value.line >= static_cast<std::uint8_t>(num_scans)) continue;
    if ((value.tag & 0x30) != 0x00 && (value.tag & 0x30) != 0x10) continue;
    ++valid_count;
    if (valid_count % point_filter_num != 0) continue;
    PointT& stored = full[i];
    stored.x = value.x;
    stored.y = value.y;
    stored.z = value.z;
    stored.intensity = value.intensity;
    stored.curvature = static_cast<float>((value.timestamp - time_base) * 1.0e-6);
    const double dist = static_cast<double>(stored.x) * stored.x + static_cast<double>(stored.y) * stored.y +
                        static_cast<double>(stored.z) * stored.z;
    if (dist < min_distance * min_distance || dist > max_distance * max_distance) continue;
    if (
      std::abs(stored.x - full[i - 1].x) <= 1.0e-7f && std::abs(stored.y - full[i - 1].y) <= 1.0e-7f &&
      std::abs(stored.z - full[i - 1].z) <= 1.0e-7f) {
      continue;
    }
    surf->push_back(stored);
  }
  logger->debug("Livox preprocessing: {} -> {} points", raw.size(), surf->size());
  return surf;
}

PointCloudT::Ptr CloudPreprocess::build_robosense_surf(const pcl::PointCloud<RobosensePoint>& raw) const {
  auto surf = boost::make_shared<PointCloudT>();
  const std::size_t size = raw.size();
  if (size == 0) return surf;
  surf->reserve(size);
  const double time_head = raw.front().timestamp;
  const float scale = 1.0e3f;
  const double omega_l = 0.361 * scan_rate;
  std::vector<bool> is_first(num_scans, true);
  std::vector<double> yaw_first(num_scans, 0.0);
  std::vector<float> yaw_last(num_scans, 0.0f);
  std::vector<float> time_last(num_scans, 0.0f);
  const bool given_offset_time = raw.back().timestamp > 0.0;
  for (std::size_t i = 0; i < size; ++i) {
    const RobosensePoint& value = raw[i];
    PointT point;
    point.x = value.x;
    point.y = value.y;
    point.z = value.z;
    point.intensity = value.intensity;
    point.curvature = static_cast<float>((value.timestamp - time_head) * scale);
    if (!given_offset_time && value.ring < static_cast<unsigned int>(num_scans)) {
      const int layer = value.ring;
      const double yaw_angle = std::atan2(point.y, point.x) * 57.2957;
      if (is_first[layer]) {
        yaw_first[layer] = yaw_angle;
        is_first[layer] = false;
        point.curvature = 0.0f;
        yaw_last[layer] = static_cast<float>(yaw_angle);
        time_last[layer] = point.curvature;
        continue;
      }
      if (yaw_angle <= yaw_first[layer]) {
        point.curvature = static_cast<float>((yaw_first[layer] - yaw_angle) / omega_l);
      } else {
        point.curvature = static_cast<float>((yaw_first[layer] - yaw_angle + 360.0) / omega_l);
      }
      if (point.curvature < time_last[layer]) point.curvature += static_cast<float>(360.0 / omega_l);
      yaw_last[layer] = static_cast<float>(yaw_angle);
      time_last[layer] = point.curvature;
    }
    const double dist = static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y +
                        static_cast<double>(point.z) * point.z;
    if (dist < min_distance * min_distance || dist > max_distance * max_distance) continue;
    if (!finite_point(point.x, point.y, point.z)) continue;
    if (i % point_filter_num == 0) surf->push_back(point);
  }
  logger->debug("Robosense preprocessing: {} -> {} points", raw.size(), surf->size());
  return surf;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  (void)stamp;
  return build_livox_surf(*raw);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  (void)stamp;
  return build_robosense_surf(*raw);
}

}  // namespace batchlio
}  // namespace asuka
