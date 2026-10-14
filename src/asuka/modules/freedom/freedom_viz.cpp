#include <asuka/modules/freedom/freedom_viz.hpp>

#include <opencv2/imgproc.hpp>

namespace asuka {

void FreedomVisualizer::set_flags(const Flags& flags_in) { flags = flags_in; }

void FreedomVisualizer::set_geometry(double sub_voxel_size_in, unsigned int voxel_depth, unsigned int block_depth) {
  sub_voxel_size = sub_voxel_size_in;
  voxel_size = sub_voxel_size * std::pow(2.0, static_cast<double>(voxel_depth));
  block_size = sub_voxel_size * std::pow(2.0, static_cast<double>(block_depth));
  half_sub_voxel_size = sub_voxel_size / 2.0;
  half_voxel_size = voxel_size / 2.0;
  half_block_size = block_size / 2.0;
  half_sub_voxel_bias = Eigen::Vector3d::Constant(half_sub_voxel_size);
  half_voxel_bias = Eigen::Vector3d::Constant(half_voxel_size);
  half_block_bias = Eigen::Vector3d::Constant(half_block_size);
}

FreedomScanRemoval::Ptr FreedomVisualizer::build_scan_removal(double stamp, const freedom::ScanMap& scan) const {
  const bool any =
    flags.scan_blocks || flags.scan_voxels || flags.clusters || flags.scan_map_range;
  if (!any) return nullptr;

  auto result = std::make_shared<FreedomScanRemoval>();
  result->stamp = stamp;

  const freedom::ScanMap::ScanBlockList& scan_blocks = scan.get_scan_blocks();

  if (flags.scan_blocks) {
    result->scan_blocks.reset(new pcl::PointCloud<pcl::PointXYZ>());
    result->scan_blocks->resize(scan_blocks.size());
    auto it = result->scan_blocks->begin();
    for (const freedom::ScanMap::ScanBlock& scan_block : scan_blocks) {
      const Eigen::Vector3d block_coord = block_size * scan_block.block_idx.cast<double>() + half_block_bias;
      it->x = block_coord.x();
      it->y = block_coord.y();
      it->z = block_coord.z();
      ++it;
    }
  }

  if (flags.scan_voxels || flags.clusters) {
    size_t total_scan_voxels = 0;
    for (const freedom::ScanMap::ScanBlock& scan_block : scan_blocks) {
      total_scan_voxels += scan_block.scan_voxels.size();
    }

    if (flags.scan_voxels) {
      result->scan_voxels.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      result->scan_voxels->resize(total_scan_voxels);
      auto it = result->scan_voxels->begin();
      for (const freedom::ScanMap::ScanBlock& scan_block : scan_blocks) {
        for (const freedom::ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels) {
          const Eigen::Vector3d voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>() + half_voxel_bias;
          it->x = voxel_coord.x();
          it->y = voxel_coord.y();
          it->z = voxel_coord.z();
          switch (scan_voxel.dynamic_level) {
            case freedom::DynamicLevel::STATIC:
              it->r = 255; it->g = 255; it->b = 255; break;
            case freedom::DynamicLevel::AGGRESSIVE_DYNAMIC:
              it->r = 0; it->g = 255; it->b = 255; break;
            case freedom::DynamicLevel::MODERATE_DYNAMIC:
              it->r = 128; it->g = 255; it->b = 0; break;
            case freedom::DynamicLevel::CONSERVATIVE_DYNAMIC:
              it->r = 255; it->g = 0; it->b = 0; break;
          }
          ++it;
        }
      }
    }

    if (flags.clusters) {
      result->clusters.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
      for (const freedom::ScanMap::ScanBlock& scan_block : scan_blocks) {
        for (const freedom::ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels) {
          if (scan_voxel.dynamic_level == freedom::DynamicLevel::STATIC) continue;
          pcl::PointXYZRGB point;
          const Eigen::Vector3d voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>() + half_voxel_bias;
          point.x = voxel_coord.x();
          point.y = voxel_coord.y();
          point.z = voxel_coord.z();
          switch (scan_voxel.dynamic_level) {
            case freedom::DynamicLevel::STATIC:
              point.r = 255; point.g = 255; point.b = 255; break;
            case freedom::DynamicLevel::AGGRESSIVE_DYNAMIC:
              point.r = 0; point.g = 255; point.b = 255; break;
            case freedom::DynamicLevel::MODERATE_DYNAMIC:
              point.r = 128; point.g = 255; point.b = 0; break;
            case freedom::DynamicLevel::CONSERVATIVE_DYNAMIC:
              point.r = 255; point.g = 0; point.b = 0; break;
          }
          result->clusters->emplace_back(point);
        }
      }
    }
  }

  if (flags.scan_map_range) {
    result->scan_map_range.valid = true;
    result->scan_map_range.min = scan.get_scan_map_min();
    result->scan_map_range.max = scan.get_scan_map_min() + scan.get_scan_map_size();
  }

  return result;
}

FreedomRaycastEnhancement::Ptr FreedomVisualizer::build_raycast_enhancement(
  double stamp, const freedom::DepthImage& image) const {
  const bool any = flags.depth_image || flags.enhanced_depth_image || flags.enhanced_pointcloud;
  if (!any) return nullptr;

  auto result = std::make_shared<FreedomRaycastEnhancement>();
  result->stamp = stamp;

  if (flags.depth_image || flags.enhanced_depth_image) {
    // Faithful port of the upstream depth-image compositing: colormap the raw
    // depth, mask out the enhancement regions and out-of-FOV pixels, then
    // composite the inpainted depths into a copy of the masked colormap.
    cv::Mat colored_depth_image;
    cv::Mat colored_inpainted_image;
    cv::applyColorMap(image.get_depth_image(), colored_depth_image, cv::COLORMAP_JET);
    cv::applyColorMap(image.get_inpainted_image(), colored_inpainted_image, cv::COLORMAP_JET);

    colored_depth_image.setTo(cv::Scalar(0, 0, 0), image.get_raycast_enhance_region());
    colored_depth_image.setTo(cv::Scalar(0, 0, 0), image.get_fov_image() == 0);

    if (flags.depth_image) {
      result->depth_image = std::make_shared<const cv::Mat>(colored_depth_image);
    }

    if (flags.enhanced_depth_image) {
      colored_inpainted_image.copyTo(colored_depth_image, image.get_eroded_raycast_enhance_region());
      result->enhanced_depth_image = std::make_shared<const cv::Mat>(colored_depth_image);
    }
  }

  if (flags.enhanced_pointcloud) {
    result->enhanced_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (const freedom::Point& point : image.get_enhanced_pointcloud()) {
      result->enhanced_pointcloud->emplace_back(point.x(), point.y(), point.z());
    }
  }

  return result;
}

FreedomMapRemoval::Ptr FreedomVisualizer::build_map_removal(double stamp, const freedom::MRMap& map) const {
  const bool any = flags.raycasted_blocks || flags.raycasted_voxels || flags.free_blocks ||
                   flags.free_voxels || flags.static_blocks || flags.static_voxels ||
                   flags.static_subvoxels || flags.static_pointcloud || flags.raycast_range ||
                   (flags.local_map_range && map.local_map_enabled());
  if (!any) return nullptr;

  auto result = std::make_shared<FreedomMapRemoval>();
  result->stamp = stamp;

  const unsigned int block_idx_size = map.getVoxel2blockMultiples();
  const unsigned int voxel_idx_size = map.getSubvoxel2voxelMultiples();
  const unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
  const unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();

  if (flags.raycasted_blocks) {
    result->raycasted_blocks.reset(new pcl::PointCloud<pcl::PointXYZ>());
    const freedom::MRMap::LocalRaycastedFlag& raycasted_flag = map.get_raycasted_flags();
    const Eigen::Vector3d raycast_map_bias = map.get_raycast_map_min();
    const Eigen::Vector3i raycast_map_idx_size = map.get_raycast_map_idx_size();
    Eigen::Vector3i local_block_idx(0, 0, 0);
    for (const std::atomic<bool>& flag : raycasted_flag) {
      if (flag) {
        const Eigen::Vector3d block_bias = block_size * local_block_idx.cast<double>();
        const Eigen::Vector3d point = raycast_map_bias + block_bias + half_block_bias;
        result->raycasted_blocks->emplace_back(point.x(), point.y(), point.z());
      }
      freedom::incrementIdx(local_block_idx, raycast_map_idx_size);
    }
  }

  if (flags.raycasted_voxels) {
    result->raycasted_voxels.reset(new pcl::PointCloud<pcl::PointXYZ>());
    const freedom::MRMap::LocalRaycastedFlag& raycasted_flag = map.get_raycasted_flags();
    const freedom::MRMap::LocalRaycastBlockGrid& raycast_blocks = map.get_raycast_blocks();
    const Eigen::Vector3d raycast_map_bias = map.get_raycast_map_min();
    const Eigen::Vector3i raycast_map_idx_size = map.get_raycast_map_idx_size();
    auto block_it = raycast_blocks.begin();
    Eigen::Vector3i local_block_idx(0, 0, 0);
    for (const std::atomic<bool>& flag : raycasted_flag) {
      if (flag) {
        const Eigen::Vector3d block_bias = block_size * local_block_idx.cast<double>();
        Eigen::Vector3i local_voxel_idx(0, 0, 0);
        for (const std::atomic<std::uint64_t>& voxel : block_it->trversed_voxels) {
          for (unsigned int bit = 0; bit < 64; ++bit) {
            if (voxel & (1ULL << bit)) {
              const Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
              const Eigen::Vector3d point = raycast_map_bias + block_bias + voxel_bias + half_voxel_bias;
              result->raycasted_voxels->emplace_back(point.x(), point.y(), point.z());
            }
            freedom::incrementIdx(local_voxel_idx, block_idx_size);
          }
        }
      }
      ++block_it;
      freedom::incrementIdx(local_block_idx, raycast_map_idx_size);
    }
  }

  if (flags.free_blocks) {
    result->free_blocks.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& block_pair : map.get_free_blocks()) {
      pcl::PointXYZRGB point;
      point.x = block_pair.first.x() * block_size + half_block_size;
      point.y = block_pair.first.y() * block_size + half_block_size;
      point.z = block_pair.first.z() * block_size + half_block_size;
      if (block_pair.second.is_free()) {
        point.r = 0; point.g = 255; point.b = 0;
      } else {
        point.r = 255; point.g = 255; point.b = 255;
      }
      result->free_blocks->emplace_back(point);
    }
  }

  if (flags.free_voxels) {
    result->free_voxels.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& block_pair : map.get_free_blocks()) {
      const Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
      const freedom::FreeBlock& free_block = block_pair.second;
      Eigen::Vector3i local_voxel_idx(0, 0, 0);
      for (unsigned int i = 0; i < voxel_num; ++i) {
        if (free_block.is_free() || free_block.getFreeVoxel(i).is_free) {
          const Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
          const Eigen::Vector3d point = block_bias + voxel_bias + half_voxel_bias;
          result->free_voxels->emplace_back(point.x(), point.y(), point.z());
        }
        freedom::incrementIdx(local_voxel_idx, block_idx_size);
      }
    }
  }

  if (flags.static_blocks) {
    result->static_blocks.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& block_pair : map.get_static_blocks()) {
      result->static_blocks->emplace_back(
        block_pair.first.x() * block_size + half_block_size,
        block_pair.first.y() * block_size + half_block_size,
        block_pair.first.z() * block_size + half_block_size);
    }
  }

  if (flags.static_voxels) {
    result->static_voxels.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& block_pair : map.get_static_blocks()) {
      const Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
      const freedom::StaticBlock& static_block = block_pair.second;
      Eigen::Vector3i local_voxel_idx(0, 0, 0);
      for (unsigned int i = 0; i < voxel_num; ++i) {
        if (static_block.is_static_voxel_allocated(i)) {
          const freedom::StaticVoxel& static_voxel = static_block.getStaticVoxel(i);
          if (static_voxel.static_occ_count > 0) {
            const Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
            const Eigen::Vector3d point = block_bias + voxel_bias + half_voxel_bias;
            result->static_voxels->emplace_back(point.x(), point.y(), point.z());
          }
        }
        freedom::incrementIdx(local_voxel_idx, block_idx_size);
      }
    }
  }

  if (flags.static_subvoxels || flags.static_pointcloud) {
    if (flags.static_subvoxels) {
      result->static_subvoxels.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
    if (flags.static_pointcloud) {
      result->static_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (const auto& block_pair : map.get_static_blocks()) {
      const Eigen::Vector3d block_bias = block_size * block_pair.first.cast<double>();
      const freedom::StaticBlock& static_block = block_pair.second;
      Eigen::Vector3i local_voxel_idx(0, 0, 0);
      for (unsigned int i = 0; i < voxel_num; ++i) {
        if (!static_block.is_static_voxel_allocated(i)) {
          freedom::incrementIdx(local_voxel_idx, block_idx_size);
          continue;
        }
        const freedom::StaticVoxel& static_voxel = static_block.getStaticVoxel(i);
        const Eigen::Vector3d voxel_bias = voxel_size * local_voxel_idx.cast<double>();
        Eigen::Vector3i local_subvoxel_idx(0, 0, 0);
        for (unsigned int j = 0; j < sub_voxel_num; ++j) {
          if (static_voxel.scan_in_subvoxel[j] != freedom::StaticVoxel::NOT_A_SCAN) {
            if (flags.static_subvoxels) {
              const Eigen::Vector3d subvoxel_bias = sub_voxel_size * local_subvoxel_idx.cast<double>();
              const Eigen::Vector3d point = block_bias + voxel_bias + subvoxel_bias + half_sub_voxel_bias;
              pcl::PointXYZRGB point_colored;
              point_colored.x = point.x();
              point_colored.y = point.y();
              point_colored.z = point.z();
              switch (static_voxel.dynamic_level[j]) {
                case freedom::DynamicLevel::STATIC:
                  point_colored.r = 255; point_colored.g = 255; point_colored.b = 255; break;
                case freedom::DynamicLevel::AGGRESSIVE_DYNAMIC:
                  point_colored.r = 0; point_colored.g = 255; point_colored.b = 255; break;
                case freedom::DynamicLevel::MODERATE_DYNAMIC:
                  point_colored.r = 128; point_colored.g = 255; point_colored.b = 0; break;
                case freedom::DynamicLevel::CONSERVATIVE_DYNAMIC:
                  point_colored.r = 255; point_colored.g = 0; point_colored.b = 0; break;
              }
              result->static_subvoxels->emplace_back(point_colored);
            }
            if (flags.static_pointcloud && static_voxel.dynamic_level[j] == freedom::DynamicLevel::STATIC) {
              const Eigen::Vector3d point = block_bias + voxel_bias + static_voxel.points[j].cast<double>();
              result->static_pointcloud->emplace_back(point.x(), point.y(), point.z());
            }
          }
          freedom::incrementIdx(local_subvoxel_idx, voxel_idx_size);
        }
        freedom::incrementIdx(local_voxel_idx, block_idx_size);
      }
    }
  }

  if (flags.raycast_range) {
    result->raycast_range.valid = true;
    result->raycast_range.min = map.get_raycast_map_min();
    result->raycast_range.max = map.get_raycast_map_max();
  }

  if (flags.local_map_range && map.local_map_enabled()) {
    result->local_map_range.valid = true;
    result->local_map_range.min = map.get_local_map_min();
    result->local_map_range.max = map.get_local_map_max();
  }

  return result;
}

}  // namespace asuka
