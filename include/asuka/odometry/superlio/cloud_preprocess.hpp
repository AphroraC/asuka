#pragma once

#include <memory>

#include <pcl/point_cloud.h>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace superlio {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();
  ~CloudPreprocess() override = default;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;

private:
  float min_distance_squared{4.0f};
  float max_distance_squared{3600.0f};
  int filter_rate{3};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace superlio
}  // namespace asuka
