#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlio {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();
  ~CloudPreprocess() override = default;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;

private:
  double min_distance{0.5};
  int point_filter_num{1};
  int num_scans{16};
  int scan_rate{10};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace fastlio
}  // namespace asuka
