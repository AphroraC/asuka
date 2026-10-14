#pragma once

// Result payloads of the freedom extension (online dynamic object removal,
// RA-L 2025 FreeDOM), shared with observers through the dedicated
// on_freedom_* callback slots. Every payload is freshly allocated per
// emission and never mutated afterwards, so observers may hold it for as
// long as they like (KeyFrame cloud contract). Null cloud fields mean the
// corresponding visualization stream is disabled in the freedom config.
//
// OpenCV is an unconditional dependency of asuka: the depth-image results
// carry cv::Mat behind a const-owned handle (asuka::ImageData semantics --
// shared, never deep-copied). cv::Mat's const does not reach the pixel
// buffer (a header copy or ROI still aliases it), so consumers must not
// create writable views; they materialize the images into sensor_msgs
// exactly once at the ROS boundary.


#include <memory>

#include <opencv2/core.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace asuka {

// Axis-aligned box used to publish the scan/raycast/local map range markers.
struct FreedomBox {
  bool valid{false};
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
};

// scan_removal stage result: the current-scan block/voxel decomposition with
// the per-voxel dynamic classification (STATIC / AGGRESSIVE / MODERATE /
// CONSERVATIVE_DYNAMIC; the RGB fields carry the classification palette).
struct FreedomScanRemoval {
  using Ptr = std::shared_ptr<FreedomScanRemoval>;
  using ConstPtr = std::shared_ptr<const FreedomScanRemoval>;

  double stamp{0.0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_blocks{nullptr};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_voxels{nullptr};
  // non-static voxels only
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters{nullptr};
  FreedomBox scan_map_range;
};

// raycast_enhancement stage result: the depth image and the inpainted
// variant with the enhancement regions composited in (both bgr8, ready for
// publishing), plus the synthesized free-space points (world frame).
struct FreedomRaycastEnhancement {
  using Ptr = std::shared_ptr<FreedomRaycastEnhancement>;
  using ConstPtr = std::shared_ptr<const FreedomRaycastEnhancement>;

  double stamp{0.0};
  // Payload immutability is typed: the Mats travel behind const-owned
  // handles (shared with the producer's staging buffers, never deep-copied).
  std::shared_ptr<const cv::Mat> depth_image{nullptr};
  std::shared_ptr<const cv::Mat> enhanced_depth_image{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr enhanced_pointcloud{nullptr};
};

// map_removal stage result: the global free/static space structures after
// the current scan was integrated.
struct FreedomMapRemoval {
  using Ptr = std::shared_ptr<FreedomMapRemoval>;
  using ConstPtr = std::shared_ptr<const FreedomMapRemoval>;

  double stamp{0.0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr raycasted_blocks{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr raycasted_voxels{nullptr};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr free_blocks{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_voxels{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr static_blocks{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr static_voxels{nullptr};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_subvoxels{nullptr};
  // the actual static map points (STATIC subvoxels only)
  pcl::PointCloud<pcl::PointXYZ>::Ptr static_pointcloud{nullptr};
  FreedomBox raycast_range;
  FreedomBox local_map_range;
};

}  // namespace asuka

