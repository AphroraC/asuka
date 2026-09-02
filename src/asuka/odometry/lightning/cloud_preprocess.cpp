#include <asuka/odometry/lightning/cloud_preprocess.hpp>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace lightning {

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  min_distance = odometry_config.param<double>("preprocess", "min_distance", 0.1);
  point_filter_num = odometry_config.param<int>("preprocess", "point_filter_num", 4);
  num_scans = odometry_config.param<int>("preprocess", "scan_line", 32);

  height_max = odometry_config.param<double>("roi", "height_max", 1.0);
  height_min = odometry_config.param<double>("roi", "height_min", -1.0);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  (void)stamp;
  output->clear();
  if (raw->empty()) return output;

  const int plsize = static_cast<int>(raw->size());
  const double timebase = raw->points[0].timestamp;
  const double blind2 = min_distance * min_distance;

  output->reserve(plsize);

  for (int i = 0; i < plsize; ++i) {
    if (i % point_filter_num != 0) continue;

    const auto& pt = raw->points[i];

    if (pt.z < height_min || pt.z > height_max) continue;

    const double range2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (range2 <= blind2) continue;

    if (i > 0) {
      const auto& prev = raw->points[i - 1];
      if (std::fabs(pt.x - prev.x) < 1e-7 && std::fabs(pt.y - prev.y) < 1e-7 && std::fabs(pt.z - prev.z) < 1e-7)
        continue;
    }

    PointT p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    p.curvature = static_cast<float>((pt.timestamp - timebase) / 1e6);

    output->push_back(p);
  }

  output->width = static_cast<int>(output->size());
  output->height = 1;
  output->is_dense = false;

  logger->debug("Livox preprocessed {} points from {} input points", output->size(), plsize);
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  output->clear();
  if (raw->empty()) return output;

  const int plsize = static_cast<int>(raw->size());
  const double blind2 = min_distance * min_distance;

  output->reserve(plsize);

  for (int i = 0; i < plsize; ++i) {
    if (i % point_filter_num != 0) continue;

    const auto& pt = raw->points[i];

    const double range2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (range2 <= blind2) continue;

    if (pt.z < height_min || pt.z > height_max) continue;

    PointT p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    p.curvature = static_cast<float>((pt.timestamp - stamp) * 1e3);

    output->push_back(p);
  }

  output->width = static_cast<int>(output->size());
  output->height = 1;
  output->is_dense = false;

  logger->debug("Robosense preprocessed {} points from {} input points", output->size(), plsize);
  return output;
}

}  // namespace lightning
}  // namespace asuka
