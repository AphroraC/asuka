#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

class RvizViewer : public ExtensionModule {
public:
  RvizViewer();
  ~RvizViewer() override;
  void stop() override;

private:
  void on_new_frame(const KeyFrame::ConstPtr& frame);
  void on_odometry_imu(const KeyFrame::ConstPtr& frame);
  void on_odometry_opt(const KeyFrame::ConstPtr& frame);
  void publish_odometry(const KeyFrame::ConstPtr& frame, ros::Publisher& publisher);
  void publish_pointcloud(
    double stamp,
    const std::string& frame_id,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    ros::Publisher& publisher);
  void publish_tf(const KeyFrame::ConstPtr& frame);
  void prepare_clouds(
    const KeyFrame::ConstPtr& frame,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_imu,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world);

  int on_new_frame_id{-1};
  int on_odometry_imu_id{-1};
  int on_odometry_opt_id{-1};

  ros::NodeHandle nh{"~"};
  ros::Publisher odometry_pub;
  ros::Publisher odometry_imu_pub;
  ros::Publisher odometry_opt_pub;
  ros::Publisher cloud_imu_pub;
  ros::Publisher cloud_world_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster{nullptr};

  std::string world_frame_id{"world"};
  std::string imu_frame_id{"imu"};
  bool enable_odometry_publish{true};
  bool enable_odometry_imu_publish{true};
  bool enable_odometry_opt_publish{true};
  bool enable_cloud_imu_publish{true};
  bool enable_cloud_world_publish{true};
  bool enable_tf_publish{true};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka
