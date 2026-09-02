#include <asuka/odometry/fastlio/cloud_preprocess.hpp>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace fastlio {

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");
  const Config config(GlobalConfig::get_config_path("config_odometry"));
  min_distance = config.param<double>("preprocess", "min_distance", 0.5);
  point_filter_num = std::max(1, config.param<int>("preprocess", "point_filter_num", 1));
  num_scans = config.param<int>("preprocess", "scan_line", 16);
  scan_rate = config.param<int>("preprocess", "scan_rate", 10);

  logger->debug(
    "fastlio::CloudPreprocess: min_distance={} filt_num={} scans={} rate={}Hz",
    min_distance,
    point_filter_num,
    num_scans,
    scan_rate);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  const double blind2 = min_distance * min_distance;
  const int plsize = static_cast<int>(raw->size());
  const double timebase = raw->points[0].timestamp;

  output->resize(plsize);

  PointT prev_assigned;
  int out_idx = 0;
  for (int i = 1; i < plsize; ++i) {
    const auto& pt = raw->points[i];

    if ((pt.tag & 0x30) != 0x10 && (pt.tag & 0x30) != 0x00) continue;
    if (pt.line >= num_scans) continue;

    if (i % point_filter_num != 0) continue;

    PointT assigned;
    assigned.x = pt.x;
    assigned.y = pt.y;
    assigned.z = pt.z;
    assigned.intensity = pt.intensity;
    assigned.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);

    if (!(std::fabs(assigned.x - prev_assigned.x) > 1e-7f || std::fabs(assigned.y - prev_assigned.y) > 1e-7f ||
          std::fabs(assigned.z - prev_assigned.z) > 1e-7f)) {
      prev_assigned = assigned;
      continue;
    }
    prev_assigned = assigned;

    if (assigned.x * assigned.x + assigned.y * assigned.y + assigned.z * assigned.z <= blind2) continue;

    output->points[out_idx++] = assigned;
  }

  output->resize(out_idx);
  output->width = out_idx;

  logger->debug("Livox preprocessed {} points from {} input points", output->size(), plsize);
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  const double blind2 = min_distance * min_distance;
  const int plsize = static_cast<int>(raw->size());
  const double timebase = raw->points[0].timestamp;

  const bool given_offset_time = raw->points[plsize - 1].timestamp > 0;
  const double omega_l = 3.61;
  std::vector<bool> is_first(num_scans, true);
  std::vector<double> yaw_fp(num_scans, 0.0);
  std::vector<float> time_last(num_scans, 0.0f);

  output->reserve(plsize);

  for (int i = 0; i < plsize; ++i) {
    const auto& pt = raw->points[i];

    PointT added;
    added.x = pt.x;
    added.y = pt.y;
    added.z = pt.z;
    added.intensity = pt.intensity;
    added.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);

    if (!given_offset_time) {
      const int layer = pt.ring;
      const double yaw_angle = std::atan2(added.y, added.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added.curvature = 0.0f;
        time_last[layer] = added.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[layer]) {
        added.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle) / omega_l);
      } else {
        added.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle + 360.0) / omega_l);
      }

      if (added.curvature < time_last[layer]) {
        added.curvature += static_cast<float>(360.0 / omega_l);
      }

      time_last[layer] = added.curvature;
    }

    if (i % point_filter_num != 0) continue;

    if (added.x * added.x + added.y * added.y + added.z * added.z > blind2) {
      output->push_back(added);
    }
  }

  logger->debug("Robosense preprocessed {} points from {} input points", output->size(), plsize);
  return output;
}

}  // namespace fastlio
}  // namespace asuka
