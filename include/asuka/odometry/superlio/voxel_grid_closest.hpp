#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include <tsl/robin_hood.h>

#include <asuka/core/types.hpp>

namespace asuka {
namespace superlio {

template <typename PointType>
class VoxelGridClosest {
private:
  using Point = PointType;
  using PointCloud = pcl::PointCloud<Point>;
  using CloudPtr = typename PointCloud::Ptr;

  CloudPtr cloud;
  float voxel_size = 0.5f;
  float inv_voxel_size = 2.0f;
  robin_hood::unordered_flat_map<std::size_t, std::size_t> voxel_map;

  std::vector<Point, Eigen::aligned_allocator<Point>> points;
  std::vector<float> dist2;
  const Eigen::Vector3i offset = Eigen::Vector3i(1000, 1000, 1000);

public:
  VoxelGridClosest() {
    dist2.reserve(10000);
    points.reserve(10000);
    voxel_map.reserve(10000);
  }

  void set_leaf_size(float lx) {
    voxel_size = lx;
    inv_voxel_size = 1.0f / lx;
  }

  void set_input_cloud(const CloudPtr& input_cloud) {
    cloud = input_cloud;
  }

  void filter(CloudPtr& output) {
    voxel_map.clear();
    dist2.clear();
    points.clear();

    for (const auto& pt : cloud->points) {
      Eigen::Vector3f pf = pt.getVector3fMap();
      Eigen::Vector3i idx = (pf * inv_voxel_size).array().round().cast<int>();
      Eigen::Vector3f center = voxel_size * idx.cast<float>();
      float d2 = (pf - center).squaredNorm();

      idx += offset;
      const std::size_t key = ((std::size_t(idx[2])) << 30) | ((std::size_t(idx[1])) << 15) |
                              (std::size_t(idx[0]));

      auto it = voxel_map.find(key);
      if (it == voxel_map.end()) {
        voxel_map.emplace(key, points.size());
        points.push_back(pt);
        dist2.push_back(d2);
      } else if (d2 < dist2[it->second]) {
        points[it->second] = pt;
        dist2[it->second] = d2;
      }
    }

    output->points.swap(points);
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    output->header = cloud->header;
  }
};

}  // namespace superlio
}  // namespace asuka
