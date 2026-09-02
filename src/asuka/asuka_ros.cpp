#include <asuka/asuka_ros.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <ros/package.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {

AsukaROS::AsukaROS(ros::NodeHandle& nh) : nh(nh) {
  const std::string config_dir = ros::package::getPath("asuka") + "/config";
  GlobalConfig::instance(config_dir, true);

  logger = create_module_logger("ros");

  Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  const std::string so_name = odometry_config.param<std::string>("odometry", "so_name", "");
  odometry = OdometryEstimation::load_module(so_name);
  if (!odometry) {
    throw std::runtime_error("failed to load odometry module: " + so_name);
  }
  cloud_preprocess = odometry->cloud_preprocess();
  async = std::make_unique<AsyncOdometryEstimation>(odometry);

  Config ros_config(GlobalConfig::get_config_path("config_ros"));
  const std::string lidar_type_str = ros_config.param<std::string>("ros", "lidar_type", "livox");
  lidar_type = parse_lidar_type(lidar_type_str);
  acc_scale = ros_config.param<double>("ros", "acc_scale", 1.0);
  imu_time_offset = ros_config.param<double>("ros", "imu_time_offset", 0.0);
  time_offset_lidar_to_imu = ros_config.param<double>("ros", "time_offset_lidar_to_imu", 0.0);
  map_saving_path = ros_config.param<std::string>("ros", "map_saving_path", "");

  load_extensions();
}

AsukaROS::~AsukaROS() {
  // Stop every producer before destroying anything: the odometry worker,
  // then all module threads. Once stop() has returned everywhere there are
  // no in-flight slot emissions left, so the modules can unsubscribe from
  // the callback slots and destruct without racing an emitter.
  if (async) async->stop();
  for (auto& ext : extension_modules) {
    ext->stop();
  }
  for (auto& ext : extension_modules) {
    ext->at_exit(map_saving_path);
  }
}

void AsukaROS::insert_imu(const sensor_msgs::Imu::ConstPtr& msg) {
  ImuData::Ptr imu = std::make_shared<ImuData>();
  imu->stamp = msg->header.stamp.toSec();
  handle_loop_back(imu->stamp, FrameId::IMU);

  imu->linear_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu->angular_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  imu->stamp += imu_time_offset;
  imu->linear_acc *= acc_scale;
  async->insert_imu(imu);
}

void AsukaROS::insert_frame(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  const double stamp = cloud->header.stamp.toSec();
  switch (lidar_type) {
    case LidarType::LIVOX: {
      pcl::PointCloud<LivoxPoint>::Ptr raw(new pcl::PointCloud<LivoxPoint>());
      pcl::fromROSMsg(*cloud, *raw);
      insert_frame(stamp, raw);
      break;
    }
    case LidarType::ROBOSENSE: {
      pcl::PointCloud<RobosensePoint>::Ptr raw(new pcl::PointCloud<RobosensePoint>());
      pcl::fromROSMsg(*cloud, *raw);
      insert_frame(stamp, raw);
      break;
    }
  }
}

void AsukaROS::insert_frame(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) return;
  async->insert_frame(adjusted_stamp, processed);
}

void AsukaROS::insert_frame(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) return;
  async->insert_frame(adjusted_stamp, processed);
}

int AsukaROS::workload() const {
  return async ? async->workload() : 0;
}

void AsukaROS::wait(bool auto_quit) {
  ros::WallRate rate(100);
  while (ros::ok()) {
    rate.sleep();
    if (auto_quit && async && async->workload() == 0) break;
  }
}

void AsukaROS::save(const std::string& path) {
  // Fully stop the async worker before touching the map: workload() only
  // reflects the input buffers, so the worker may still be inside
  // process_scan() writing map_cloud while save() reads it (data race /
  // heap corruption). The worker is joined here; the destructor's stop()
  // is idempotent.
  if (async) async->stop();
  auto cloud = odometry ? odometry->save_map() : nullptr;
  if (cloud && !cloud->empty() && !path.empty()) {
    std::filesystem::path fs_path(path);
    std::string file_path;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream stamp;
    stamp << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
    std::string filename = "mapping_" + stamp.str() + ".pcd";
    if (fs_path.has_extension()) {
      file_path = path;
    } else {
      std::filesystem::create_directories(path);
      file_path = path + "/" + filename;
    }
    try {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
      cloud_xyzi->resize(cloud->size());
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        cloud_xyzi->points[i].x = cloud->points[i].x;
        cloud_xyzi->points[i].y = cloud->points[i].y;
        cloud_xyzi->points[i].z = cloud->points[i].z;
        cloud_xyzi->points[i].intensity = cloud->points[i].intensity;
      }
      pcl::io::savePCDFileBinary(file_path, *cloud_xyzi);
      logger->info("saved map ({} points) to {}", cloud_xyzi->size(), file_path);
    } catch (const pcl::IOException& e) {
      logger->warn("failed to save map: {}", e.what());
    }
  }
}

void AsukaROS::load_extensions() {
  Config extensions_config(GlobalConfig::get_config_path("config_extensions"));
  const auto so_names = extensions_config.param<std::vector<std::string>>(
    "extensions",
    "loading_extension_modules",
    std::vector<std::string>());
  for (const auto& so_name : so_names) {
    auto ext = ExtensionModule::load_module(so_name);
    if (ext) {
      extension_modules.push_back(ext);
    }
  }
}

void AsukaROS::handle_loop_back(double stamp, const FrameId& frame_id) {
  if (frame_id == FrameId::LIDAR) {
    if (stamp < last_timestamp_lidar) {
      logger->warn("lidar loop back detected: {} -> {}", last_timestamp_lidar, stamp);
    }
    last_timestamp_lidar = stamp;
  } else if (frame_id == FrameId::IMU) {
    if (stamp < last_timestamp_imu) {
      logger->warn("imu loop back detected: {} -> {}", last_timestamp_imu, stamp);
    }
    last_timestamp_imu = stamp;
  }
}

}  // namespace asuka
