#pragma once

#include <memory>

#include <pcl/point_cloud.h>

#include <asuka/core/types.hpp>

namespace asuka {

class CloudPreprocess {
public:
  virtual ~CloudPreprocess() = default;

  virtual PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
    (void)stamp;
    (void)raw;
    return nullptr;
  }

  virtual PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
    (void)stamp;
    (void)raw;
    return nullptr;
  }

protected:
  CloudPreprocess() = default;
};

}  // namespace asuka
