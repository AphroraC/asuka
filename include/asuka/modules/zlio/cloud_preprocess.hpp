#pragma once

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>

namespace asuka {
namespace zlio {

// 纯格式转换：xyz/intensity 透传、逐点时间换算为 curvature 毫秒（扫描时基）。
// 上游 zlio 只在内部做量程过滤（minimum/maximum_range），此处不添加任何
// 盲区/线数/抽稀过滤——与上游语义一致（parity）。
class CloudPreprocess : public asuka::CloudPreprocess {
public:
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) override;
};

}  // namespace zlio
}  // namespace asuka
