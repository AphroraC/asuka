#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace lightning {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();
  ~CloudPreprocess() override = default;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;

private:
  double min_distance{0.01};
  int point_filter_num{1};
  int num_scans{6};
  float height_max{1.0};
  float height_min{-1.0};

  PointCloudT::Ptr output{new PointCloudT};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace lightning
}  // namespace asuka
