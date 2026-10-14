#include <asuka/modules/freedom/freedom_module.hpp>

#include <asuka/modules/freedom/freedom_callbacks.hpp>

#include <cmath>
#include <filesystem>

#include <asuka/modules/freedom/freedom_types.hpp>

namespace asuka {

FreedomModule::FreedomModule() {
  logger = create_module_logger("freedom");

  const Config config = load_config("config_extensions");

  // ---- module-level parameters ----
  enable_map_saving = config.param<bool>("freedom", "enable_map_saving", false);
  map_saving_path = config.param<std::string>("freedom", "map_saving_path", "");
  int num_threads = config.param<int>("freedom", "num_threads", 8);

  // ---- upstream FreeDOM::Config, mirroring freenode.cpp get_params ----
  freedom::FreeDOM::Config map_config;
  map_config.sensor_min_range = config.param_nested<double>({"freedom", "sensor"}, "min_range", 0.0);
  map_config.sensor_max_range = config.param_nested<double>({"freedom", "sensor"}, "max_range", 50.0);
  map_config.sensor_min_z = config.param_nested<double>({"freedom", "sensor"}, "min_z", -20.0);
  map_config.sensor_max_z = config.param_nested<double>({"freedom", "sensor"}, "max_z", 20.0);

  map_config.sub_voxel_size = config.param_nested<double>({"freedom", "map"}, "sub_voxel_size", 0.1);
  map_config.voxel_depth = config.param_nested<int>({"freedom", "map"}, "voxel_depth", 2);
  map_config.block_depth = config.param_nested<int>({"freedom", "map"}, "block_depth", 5);
  map_config.enable_local_map = config.param_nested<bool>({"freedom", "map"}, "enable_local_map", false);
  map_config.local_map_range = config.param_nested<double>({"freedom", "map"}, "local_map_range", 100.0);
  map_config.local_map_min_z = config.param_nested<double>({"freedom", "map"}, "local_map_min_z", -20.0);
  map_config.local_map_max_z = config.param_nested<double>({"freedom", "map"}, "local_map_max_z", 20.0);
  map_config.raycast_max_range = config.param_nested<double>({"freedom", "map"}, "raycast_max_range", 100.0);
  map_config.raycast_min_z = config.param_nested<double>({"freedom", "map"}, "raycast_min_z", -20.0);
  map_config.raycast_max_z = config.param_nested<double>({"freedom", "map"}, "raycast_max_z", 20.0);
  map_config.counts_to_free = config.param_nested<int>({"freedom", "map"}, "counts_to_free", 6);
  map_config.counts_to_revert = config.param_nested<int>({"freedom", "map"}, "counts_to_revert", 20);
  map_config.conservative_connectivity = config.param_nested<int>({"freedom", "map"}, "conservative_connectivity", 26);
  map_config.aggressive_connectivity = config.param_nested<int>({"freedom", "map"}, "aggressive_connectivity", 124);
  map_config.num_threads = num_threads;

  map_config.enable_raycast_enhancement =
    config.param_nested<bool>({"freedom", "raycast_enhancement"}, "enable_raycast_enhancement", false);
  if (map_config.enable_raycast_enhancement) {
    // FOV angles are configured in degrees (upstream yaml convention), the
    // pipeline wants radians.
    map_config.lidar_horizon_fov =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "lidar_horizon_fov_degree", 360.0) * M_PI / 180.0;
    map_config.lidar_vertical_fov_upper =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "lidar_vertical_fov_upper_degree", 16.0) * M_PI /
      180.0;
    map_config.lidar_vertical_fov_lower =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "lidar_vertical_fov_lower_degree", -54.4) * M_PI /
      180.0;
    map_config.depth_image_vertical_lines =
      config.param_nested<int>({"freedom", "raycast_enhancement"}, "depth_image_vertical_lines", 64);
    map_config.depth_image_min_range =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "depth_image_min_range", 0.0);
    map_config.max_raycast_enhancement_range =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "max_raycast_enhancement_range", 50.0);
    map_config.raycast_enhancement_depth_margin =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "raycast_enhancement_depth_margin", 0.0);
    map_config.inpaint_size = config.param_nested<int>({"freedom", "raycast_enhancement"}, "inpaint_size", 3);
    map_config.erosion_size = config.param_nested<int>({"freedom", "raycast_enhancement"}, "erosion_size", 0);
    map_config.min_raycast_enhancement_area =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "min_raycast_enhancement_area", 0.0);
    map_config.depth_image_top_margin =
      config.param_nested<double>({"freedom", "raycast_enhancement"}, "depth_image_top_margin", 0.0);
    map_config.learn_fov = config.param_nested<bool>({"freedom", "raycast_enhancement"}, "learn_fov", false);
    map_config.enable_fov_mask = config.param_nested<bool>({"freedom", "raycast_enhancement"}, "enable_fov_mask", false);
    if (map_config.learn_fov || map_config.enable_fov_mask) {
      map_config.fov_mask_path =
        config.param_nested<std::string>({"freedom", "raycast_enhancement"}, "fov_mask_path", std::string());
      if (map_config.fov_mask_path.empty()) {
        throw std::runtime_error("freedom: fov_mask_path is required when learn_fov/enable_fov_mask is on");
      }
    }
  }

  freedom.set_params(map_config);

  // ---- visualization stream flags ----
  FreedomVisualizer::Flags viz_flags;
  viz_flags.scan_blocks = config.param_nested<bool>({"freedom", "publish"}, "scan_blocks", false);
  viz_flags.scan_voxels = config.param_nested<bool>({"freedom", "publish"}, "scan_voxels", true);
  viz_flags.clusters = config.param_nested<bool>({"freedom", "publish"}, "clusters", false);
  viz_flags.depth_image = config.param_nested<bool>({"freedom", "publish"}, "depth_image", true);
  viz_flags.enhanced_depth_image = config.param_nested<bool>({"freedom", "publish"}, "enhanced_depth_image", true);
  viz_flags.enhanced_pointcloud = config.param_nested<bool>({"freedom", "publish"}, "enhanced_pointcloud", true);
  viz_flags.raycasted_blocks = config.param_nested<bool>({"freedom", "publish"}, "raycasted_blocks", false);
  viz_flags.raycasted_voxels = config.param_nested<bool>({"freedom", "publish"}, "raycasted_voxels", false);
  viz_flags.free_blocks = config.param_nested<bool>({"freedom", "publish"}, "free_blocks", false);
  viz_flags.free_voxels = config.param_nested<bool>({"freedom", "publish"}, "free_voxels", false);
  viz_flags.static_blocks = config.param_nested<bool>({"freedom", "publish"}, "static_blocks", false);
  viz_flags.static_voxels = config.param_nested<bool>({"freedom", "publish"}, "static_voxels", false);
  viz_flags.static_subvoxels = config.param_nested<bool>({"freedom", "publish"}, "static_subvoxels", false);
  viz_flags.static_pointcloud = config.param_nested<bool>({"freedom", "publish"}, "static_pointcloud", true);
  viz_flags.scan_map_range = config.param_nested<bool>({"freedom", "publish"}, "scan_map_range", false);
  viz_flags.raycast_range = config.param_nested<bool>({"freedom", "publish"}, "raycast_range", false);
  viz_flags.local_map_range = config.param_nested<bool>({"freedom", "publish"}, "local_map_range", false);
  visualizer.set_flags(viz_flags);
  visualizer.set_geometry(map_config.sub_voxel_size, map_config.voxel_depth, map_config.block_depth);

  // ---- stage callbacks: snapshot + slot emission (same thread as the data) ----
  freedom.set_scan_removal_callback([this](const freedom::ScanMap& scan) { emit_scan_removal(scan); });
  freedom.set_raycast_enhancement_callback([this](const freedom::DepthImage& image) { emit_raycast_enhancement(image); });
  freedom.set_map_removal_callback([this](const freedom::MRMap& map) { emit_map_removal(map); });

  // ---- keyframe input + processing thread ----
  on_new_frame_sub =
    Callbacks::on_new_frame.subscribe(std::bind(&FreedomModule::on_new_frame, this, std::placeholders::_1));

  kill_switch = false;
  processing_thread_obj = JoiningThread(&FreedomModule::processing_thread, this);

  logger->info(
    "freedom module initialized (sub_voxel={} voxel_depth={} block_depth={} threads={} raycast_enhancement={})",
    map_config.sub_voxel_size, map_config.voxel_depth, map_config.block_depth, num_threads,
    map_config.enable_raycast_enhancement);
}

FreedomModule::~FreedomModule() {
  stop();
}

void FreedomModule::stop() {
  if (!kill_switch.exchange(true)) {
    queue_cv.notify_all();
  }
  processing_thread_obj.join();
}

void FreedomModule::at_exit(const std::string& dump_path) {
  // Map saving is configured in the freedom block (the AsukaROS-level
  // dump_path from config_ros belongs to the odometry frontends).
  (void)dump_path;
  if (!enable_map_saving || map_saving_path.empty()) {
    logger->debug("freedom map saving disabled");
    return;
  }
  // Upstream save_map concatenates the file names onto the path, so the
  // directory must carry a trailing separator.
  std::filesystem::path dir(map_saving_path);
  std::filesystem::create_directories(dir);
  std::string dir_with_sep = (dir / "").string();
  freedom.save_map(dir_with_sep);
  logger->info("freedom static map saved to {}", dir_with_sep);
}

void FreedomModule::on_new_frame(const KeyFrame::ConstPtr& frame) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  if (queue.full()) {
    if (++dropped_items % 100 == 1) {
      logger->warn("freedom queue overflow, dropping oldest items ({} dropped)", dropped_items);
    }
  }
  queue.push_back(frame);
  queue_cv.notify_one();
}

void FreedomModule::processing_thread() {
  while (true) {
    KeyFrame::ConstPtr frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !queue.empty() || kill_switch; });
      if (kill_switch && queue.empty()) return;
      frame = queue.front();
      queue.pop_front();
    }
    if (frame) {
      // One broken frame must not kill the module thread.
      try {
        process_frame(frame);
      } catch (const std::exception& e) {
        logger->error("freedom frame processing failed: {}", e.what());
      } catch (...) {
        logger->error("freedom frame processing failed: unknown exception");
      }
    }
  }
}

void FreedomModule::process_frame(const KeyFrame::ConstPtr& frame) {
  // Defensive: the on_new_frame contract excludes pose-only frames, but a
  // frontend regression must not feed an empty cloud into the pipeline.
  if (!frame->cloud_imu || frame->cloud_imu->empty() || frame->id < 0) return;

  // The pipeline consumes pcl::PointXYZ; only xyz is used.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize(frame->cloud_imu->size());
  for (std::size_t i = 0; i < frame->cloud_imu->size(); ++i) {
    cloud[i].x = frame->cloud_imu->points[i].x;
    cloud[i].y = frame->cloud_imu->points[i].y;
    cloud[i].z = frame->cloud_imu->points[i].z;
  }

  current_stamp = frame->stamp;
  freedom.pointcloud_integrate(cloud, frame->T_world_imu);
}

void FreedomModule::emit_scan_removal(const freedom::ScanMap& scan) {
  if (auto payload = visualizer.build_scan_removal(current_stamp, scan)) {
    FreedomCallbacks::on_freedom_scan_removal(payload);
  }
}

void FreedomModule::emit_raycast_enhancement(const freedom::DepthImage& image) {
  if (auto payload = visualizer.build_raycast_enhancement(current_stamp, image)) {
    FreedomCallbacks::on_freedom_raycast_enhancement(payload);
  }
}

void FreedomModule::emit_map_removal(const freedom::MRMap& map) {
  if (auto payload = visualizer.build_map_removal(current_stamp, map)) {
    FreedomCallbacks::on_freedom_map_removal(payload);
  }
}

}  // namespace asuka
