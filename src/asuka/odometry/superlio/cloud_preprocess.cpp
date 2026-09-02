#include "asuka/odometry/superlio/cloud_preprocess.hpp"

#include <cmath>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace superlio {

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");
  const Config config(GlobalConfig::get_config_path("config_odometry"));
  const double min_distance = config.param<double>("preprocess", "min_distance", 2.0);
  const double max_distance = config.param<double>("preprocess", "max_distance", 60.0);
  min_distance_squared = static_cast<float>(min_distance * min_distance);
  max_distance_squared = static_cast<float>(max_distance * max_distance);
  filter_rate = std::max(1, config.param<int>("preprocess", "filter_rate", 3));
  logger->info(
    "superlio::CloudPreprocess: min_distance_squared={} max_distance_squared={} filter_rate={}",
    min_distance_squared,
    max_distance_squared,
    filter_rate);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;
  if (raw->size() < 2) return output;

  const double timebase = raw->points[0].timestamp;
  output->reserve(raw->size() / filter_rate + 1);

  for (std::size_t i = 0; i < raw->size(); i += filter_rate) {
    const auto& pt = raw->points[i];
    if ((pt.tag & 0x30) != 0x10 && (pt.tag & 0x30) != 0x00) continue;

    const float dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dis <= min_distance_squared || dis >= max_distance_squared) continue;

    PointT p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    p.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);
    output->push_back(p);
  }

  logger->debug("Livox preprocessed {} points from {} input points", output->size(), raw->size());
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  const double timebase = raw->points[0].timestamp;
  output->reserve(raw->size() / filter_rate + 1);

  for (std::size_t i = 0; i < raw->size(); i += filter_rate) {
    const auto& pt = raw->points[i];

    const float dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dis <= min_distance_squared || dis >= max_distance_squared) continue;

    PointT p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    p.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);
    output->push_back(p);
  }

  logger->debug("Robosense preprocessed {} points from {} input points", output->size(), raw->size());
  return output;
}

}  // namespace superlio
}  // namespace asuka
