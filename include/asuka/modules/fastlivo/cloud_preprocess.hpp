#pragma once

#include <memory>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/modules/fastlivo/common_lib.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlivo {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) override;

private:
  std::shared_ptr<spdlog::logger> logger{nullptr};

  int point_filter_num{1};
  int num_scans{6};
  double blind{0.01};
  double blind_sqr{0.0001};
};

}  // namespace fastlivo
}  // namespace asuka
