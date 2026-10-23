#include <asuka/modules/zlio/cloud_preprocess.hpp>

namespace asuka {
namespace zlio {

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  // Livox PointCloud2 的逐点 timestamp 为纳秒绝对时刻：归约到扫描时基换算为
  // 毫秒（与 fastlio/superlio 的 livox 通路同契约）。
  const double timebase = raw->points[0].timestamp;
  output->resize(raw->size());
  for (std::size_t i = 0; i < raw->size(); ++i) {
    const auto& point = raw->points[i];
    PointT& out = output->points[i];
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    out.intensity = point.intensity;
    out.curvature = static_cast<float>((point.timestamp - timebase) / 1e6);
  }
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  // RobosensePoint::timestamp 为纳秒（已对录制 bag 实证的上游契约）。
  const double timebase = raw->points[0].timestamp;
  output->resize(raw->size());
  for (std::size_t i = 0; i < raw->size(); ++i) {
    const auto& point = raw->points[i];
    PointT& out = output->points[i];
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    out.intensity = point.intensity;
    out.curvature = static_cast<float>((point.timestamp - timebase) / 1e6);
  }
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;

  // CustomMsg 的 offset_time 是驱动给出的纳秒相对偏移。
  output->resize(raw->size());
  for (std::size_t i = 0; i < raw->size(); ++i) {
    const auto& point = raw->points[i];
    PointT& out = output->points[i];
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    out.intensity = point.intensity;
    out.curvature = static_cast<float>(point.offset_time * 1e-6);
  }
  return output;
}

}  // namespace zlio
}  // namespace asuka
