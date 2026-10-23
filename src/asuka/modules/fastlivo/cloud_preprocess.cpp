#include <asuka/modules/fastlivo/cloud_preprocess.hpp>

#include <algorithm>

#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlivo {

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");
  const Config config = load_config("config_odometry");
  blind = config.param<double>("preprocess", "blind", 0.01);
  num_scans = config.param<int>("preprocess", "scan_line", 6);
  point_filter_num = config.param<int>("preprocess", "point_filter_num", 3);
  blind_sqr = blind * blind;

  logger->debug("fastlivo::CloudPreprocess: blind={} scans={} filter_num={}", blind, num_scans, point_filter_num);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr pl_surf(new PointCloudT);
  if (raw->empty()) return pl_surf;

  const int plsize = static_cast<int>(raw->size());
  pl_surf->reserve(plsize);
  const double timebase = raw->points[0].timestamp;

  uint valid_num = 0;
  float prev_curv = 0.0f;
  for (int i = 0; i < plsize; i++) {
    const auto& pt = raw->points[i];
    if (pt.line >= num_scans) continue;

    valid_num++;

    PointT added_pt;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);

    if (i == 0)
      added_pt.curvature = std::fabs(added_pt.curvature) < 1.0f ? added_pt.curvature : 0.0f;
    else {
      added_pt.curvature = std::fabs(added_pt.curvature - prev_curv) < 1.0f ? added_pt.curvature : prev_curv + 0.004166667f;
    }
    prev_curv = added_pt.curvature;

    if (valid_num % point_filter_num == 0) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >= blind_sqr) {
        pl_surf->push_back(added_pt);
      }
    }
  }
  return pl_surf;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr pl_surf(new PointCloudT);
  if (raw->empty()) return pl_surf;

  const int plsize = static_cast<int>(raw->size());
  pl_surf->reserve(plsize);

  uint valid_num = 0;
  float prev_curv = 0.0f;
  for (int i = 0; i < plsize; i++) {
    const auto& pt = raw->points[i];
    if (pt.line >= num_scans) continue;

    valid_num++;

    PointT added_pt;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = pt.offset_time / float(1000000);  // ns -> ms

    if (i == 0)
      added_pt.curvature = std::fabs(added_pt.curvature) < 1.0f ? added_pt.curvature : 0.0f;
    else {
      added_pt.curvature = std::fabs(added_pt.curvature - prev_curv) < 1.0f ? added_pt.curvature : prev_curv + 0.004166667f;
    }
    prev_curv = added_pt.curvature;

    if (valid_num % point_filter_num == 0) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >= blind_sqr) {
        pl_surf->push_back(added_pt);
      }
    }
  }
  return pl_surf;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr pl_surf(new PointCloudT);
  if (raw->empty()) return pl_surf;

  const int plsize = static_cast<int>(raw->size());
  pl_surf->reserve(plsize);

  const double time_head = raw->points[0].timestamp;
  const bool given_offset_time = raw->points[plsize - 1].timestamp > 0;
  const double omega_l = 3.61;
  std::vector<bool> is_first(num_scans, true);
  std::vector<double> yaw_fp(num_scans, 0.0);
  std::vector<float> time_last(num_scans, 0.0f);

  for (int i = 0; i < plsize; ++i) {
    if (i % point_filter_num != 0) continue;

    const auto& pt = raw->points[i];
    const double x = pt.x, y = pt.y, z = pt.z;
    const double dist_sqr = x * x + y * y + z * z;
    const bool is_valid = (dist_sqr >= blind_sqr) && !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    if (!is_valid) continue;

    PointT added_pt;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = static_cast<float>((pt.timestamp - time_head) / 1e6);

    if (!given_offset_time) {
      const int layer = pt.ring;
      const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;

      if (layer < 0 || layer >= num_scans) continue;
      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0f;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[layer]) {
        added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle) / omega_l);
      } else {
        added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle + 360.0) / omega_l);
      }

      if (added_pt.curvature < time_last[layer]) {
        added_pt.curvature += static_cast<float>(360.0 / omega_l);
      }
      time_last[layer] = added_pt.curvature;
    }

    pl_surf->points.push_back(added_pt);
  }
  std::sort(pl_surf->points.begin(), pl_surf->points.end(), [](const PointT& a, const PointT& b) {
    return a.curvature < b.curvature;
  });
  return pl_surf;
}

}  // namespace fastlivo
}  // namespace asuka
