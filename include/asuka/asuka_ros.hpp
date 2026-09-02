#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <asuka/core/async_odometry_estimation.hpp>
#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

class AsukaROS {
public:
  explicit AsukaROS(ros::NodeHandle& nh);
  ~AsukaROS();

  void insert_imu(const sensor_msgs::Imu::ConstPtr& msg);
  void insert_frame(const sensor_msgs::PointCloud2::ConstPtr& msg);

  int workload() const;
  void wait(bool auto_quit);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<ExtensionModule>>& extensions() const { return extension_modules; }

private:
  void load_extensions();
  void handle_loop_back(double stamp, const FrameId& frame_id);
  void insert_frame(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw);
  void insert_frame(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw);

  ros::NodeHandle& nh;

  std::shared_ptr<OdometryEstimation> odometry{nullptr};
  std::unique_ptr<AsyncOdometryEstimation> async{nullptr};
  std::shared_ptr<CloudPreprocess> cloud_preprocess{nullptr};
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;

  LidarType lidar_type{LidarType::LIVOX};
  double acc_scale{1.0};
  double imu_time_offset{0.0};
  double time_offset_lidar_to_imu{0.0};
  std::string map_saving_path{};

  double last_timestamp_lidar{-1.0};
  double last_timestamp_imu{-1.0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka
