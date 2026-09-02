#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <asuka/asuka_ros.hpp>
#include <asuka/utility/config.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "asuka_rosnode");
  ros::NodeHandle nh;

  asuka::AsukaROS asuka_ros(nh);

  asuka::Config config(asuka::GlobalConfig::get_config_path("config_ros"));
  const std::string imu_topic = config.param<std::string>("ros", "imu_topic", "/livox/imu");
  const std::string lidar_topic = config.param<std::string>("ros", "lidar_topic", "/livox/lidar");
  const std::string map_saving_path = config.param<std::string>("ros", "map_saving_path", "");

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1000, [&](const sensor_msgs::Imu::ConstPtr& msg) {
    asuka_ros.insert_imu(msg);
  });

  ros::Subscriber points_sub =
    nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 50, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      asuka_ros.insert_frame(msg);
    });

  ros::spin();
  asuka_ros.wait(true);
  asuka_ros.save(map_saving_path);
  return 0;
}
