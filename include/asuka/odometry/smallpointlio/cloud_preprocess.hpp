#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace smallpointlio {

struct RawPoint {
  double timestamp{0.0};
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
};

class VoxelgridSampling {
public:
  VoxelgridSampling() = default;
  void voxelgrid_sampling(const std::vector<RawPoint>& points, std::vector<RawPoint>& downsampled, double leaf_size);

private:
  std::vector<std::pair<std::uint64_t, size_t>> coord_pt;
};

}  // namespace smallpointlio

namespace smallpointlio {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();
  ~CloudPreprocess() override = default;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) override;
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) override;

private:
  PointCloudT::Ptr build(double stamp, const pcl::PointCloud<LivoxPoint>& raw) const;
  PointCloudT::Ptr build(double stamp, const pcl::PointCloud<RobosensePoint>& raw) const;

  double min_distance{0.5};
  double max_distance{100.0};
  int point_filter_num{1};
  bool space_downsample{false};
  double space_downsample_leaf_size{0.5};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace smallpointlio
}  // namespace asuka
