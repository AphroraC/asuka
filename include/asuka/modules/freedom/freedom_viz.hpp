#pragma once

// Builds the freedom result payloads from the live scan/map structures. The
// builders run inside FreeDOM::pointcloud_integrate on the freedom module's
// processing thread -- the same thread that owns the structures -- so they
// read them without locking, exactly like the upstream Visualizer did inside
// its mapping thread. The geometry (voxel/block sizes and half-biases)
// mirrors Map::set_params, and every loop is a faithful port of the upstream
// freedom/src/visualization.cpp conversion, with the ROS message emission
// replaced by freshly allocated payload fields.

#include <asuka/modules/freedom/freedom_types.hpp>

#include <asuka/modules/freedom/depth_image.h>
#include <asuka/modules/freedom/mrmap.h>
#include <asuka/modules/freedom/scanmap.h>

namespace asuka {

class FreedomVisualizer {
public:
  // One flag per published stream, mirroring the upstream per-topic
  // getNumSubscribers() gating (a subscriber count the framework cannot see).
  // Disabled streams leave the payload field null so the consumer skips them.
  struct Flags {
    bool scan_blocks{false};
    bool scan_voxels{true};
    bool clusters{false};
    bool depth_image{true};
    bool enhanced_depth_image{true};
    bool enhanced_pointcloud{true};
    bool raycasted_blocks{false};
    bool raycasted_voxels{false};
    bool free_blocks{false};
    bool free_voxels{false};
    bool static_blocks{false};
    bool static_voxels{false};
    bool static_subvoxels{false};
    bool static_pointcloud{true};
    bool scan_map_range{false};
    bool raycast_range{false};
    bool local_map_range{false};
  };

  void set_flags(const Flags& flags);
  void set_geometry(double sub_voxel_size, unsigned int voxel_depth, unsigned int block_depth);

  // Returns a null pointer when every stream of the stage is disabled.
  FreedomScanRemoval::Ptr build_scan_removal(double stamp, const freedom::ScanMap& scan) const;
  FreedomRaycastEnhancement::Ptr build_raycast_enhancement(double stamp, const freedom::DepthImage& image) const;
  FreedomMapRemoval::Ptr build_map_removal(double stamp, const freedom::MRMap& map) const;

private:
  Flags flags;

  double sub_voxel_size{0.1};
  double voxel_size{0.4};
  double block_size{3.2};
  double half_sub_voxel_size{0.05};
  double half_voxel_size{0.2};
  double half_block_size{1.6};
  Eigen::Vector3d half_sub_voxel_bias{Eigen::Vector3d::Constant(0.05)};
  Eigen::Vector3d half_voxel_bias{Eigen::Vector3d::Constant(0.2)};
  Eigen::Vector3d half_block_bias{Eigen::Vector3d::Constant(1.6)};
};

}  // namespace asuka
