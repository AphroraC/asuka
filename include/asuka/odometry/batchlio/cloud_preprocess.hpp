#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace batchlio {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();
  ~CloudPreprocess() override = default;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;

private:
  PointCloudT::Ptr build_livox_surf(const pcl::PointCloud<LivoxPoint>& raw) const;
  PointCloudT::Ptr build_robosense_surf(const pcl::PointCloud<RobosensePoint>& raw) const;

  double min_distance{0.5};
  double max_distance{300.0};
  int point_filter_num{2};
  int num_scans{6};
  int scan_rate{10};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace batchlio
}  // namespace asuka
