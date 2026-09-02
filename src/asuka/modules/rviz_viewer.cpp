#include <asuka/modules/rviz_viewer.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace asuka {

RvizViewer::RvizViewer() {
  logger = create_module_logger("rviz");

  const Config config(GlobalConfig::get_config_path("config_extensions"));
  world_frame_id = config.param<std::string>("rviz_viewer", "world_frame_id", "world");
  imu_frame_id = config.param<std::string>("rviz_viewer", "imu_frame_id", "imu");
  enable_odometry_publish = config.param<bool>("rviz_viewer", "publish_odometry", true);
  enable_odometry_imu_publish = config.param<bool>("rviz_viewer", "publish_odometry_imu", true);
  enable_odometry_opt_publish = config.param<bool>("rviz_viewer", "publish_odometry_opt", true);
  enable_cloud_imu_publish = config.param<bool>("rviz_viewer", "publish_cloud_imu", true);
  enable_cloud_world_publish = config.param<bool>("rviz_viewer", "publish_cloud_world", true);
  enable_tf_publish = config.param<bool>("rviz_viewer", "publish_tf", true);

  odometry_pub = nh.advertise<nav_msgs::Odometry>("/asuka/odometry", 10);
  odometry_imu_pub = nh.advertise<nav_msgs::Odometry>("/asuka/odometry_imu", 100);
  odometry_opt_pub = nh.advertise<nav_msgs::Odometry>("/asuka/odometry_opt", 100);
  cloud_imu_pub = nh.advertise<sensor_msgs::PointCloud2>("/asuka/points", 10);
  cloud_world_pub = nh.advertise<sensor_msgs::PointCloud2>("/asuka/world", 10);

  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

  on_new_frame_id = Callbacks::on_new_frame.add(std::bind(&RvizViewer::on_new_frame, this, std::placeholders::_1));
  on_odometry_imu_id =
    Callbacks::on_odometry_imu.add(std::bind(&RvizViewer::on_odometry_imu, this, std::placeholders::_1));
  on_odometry_opt_id =
    Callbacks::on_odometry_opt.add(std::bind(&RvizViewer::on_odometry_opt, this, std::placeholders::_1));
}

RvizViewer::~RvizViewer() {
  Callbacks::on_new_frame.remove(on_new_frame_id);
  Callbacks::on_odometry_imu.remove(on_odometry_imu_id);
  Callbacks::on_odometry_opt.remove(on_odometry_opt_id);
}

void RvizViewer::stop() {}

void RvizViewer::on_new_frame(const KeyFrame::ConstPtr& frame) {
  if (!frame) return;
  if (enable_odometry_publish) publish_odometry(frame, odometry_pub);
  // Frontend and IMU-predicted odometry are independent streams. Once the
  // predicted stream is enabled, it is the sole owner of this TF edge.
  if (enable_tf_publish && !enable_odometry_imu_publish) publish_tf(frame);
  if (enable_cloud_imu_publish) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_imu(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>());
    prepare_clouds(frame, cloud_imu, cloud_world);
    if (enable_cloud_imu_publish) publish_pointcloud(frame->stamp, imu_frame_id, cloud_imu, cloud_imu_pub);
    if (enable_cloud_world_publish) publish_pointcloud(frame->stamp, world_frame_id, cloud_world, cloud_world_pub);
  }
}

void RvizViewer::on_odometry_imu(const KeyFrame::ConstPtr& frame) {
  if (!frame) return;
  if (enable_odometry_imu_publish) {
    publish_odometry(frame, odometry_imu_pub);
    publish_tf(frame);
  }
}

void RvizViewer::on_odometry_opt(const KeyFrame::ConstPtr& frame) {
  if (!frame) return;
  if (enable_odometry_opt_publish) {
    publish_odometry(frame, odometry_opt_pub);
  }
}

void RvizViewer::publish_odometry(const KeyFrame::ConstPtr& frame, ros::Publisher& publisher) {
  const Eigen::Vector3d pos = frame->T_world_imu.translation();
  const Eigen::Quaterniond quat(frame->T_world_imu.rotation());

  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time().fromSec(frame->stamp);
  msg.header.frame_id = world_frame_id;
  msg.child_frame_id = imu_frame_id;

  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();
  msg.pose.pose.orientation.x = quat.x();
  msg.pose.pose.orientation.y = quat.y();
  msg.pose.pose.orientation.z = quat.z();
  msg.pose.pose.orientation.w = quat.w();

  msg.twist.twist.linear.x = frame->v_world_imu.x();
  msg.twist.twist.linear.y = frame->v_world_imu.y();
  msg.twist.twist.linear.z = frame->v_world_imu.z();

  publisher.publish(msg);

  logger->trace("publish odometry stamp={:.3f} pos=({:.2f},{:.2f},{:.2f})", frame->stamp, pos.x(), pos.y(), pos.z());
}

void RvizViewer::publish_pointcloud(
  double stamp,
  const std::string& frame_id,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
  ros::Publisher& publisher) {
  if (cloud->empty()) return;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time().fromSec(stamp);
  msg.header.frame_id = frame_id;
  publisher.publish(msg);

  logger->trace("publish cloud {}pts frame={}", cloud->size(), frame_id);
}

void RvizViewer::publish_tf(const KeyFrame::ConstPtr& frame) {
  const Eigen::Vector3d pos = frame->T_world_imu.translation();
  const Eigen::Quaterniond quat(frame->T_world_imu.rotation());

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time().fromSec(frame->stamp);
  tf_msg.header.frame_id = world_frame_id;
  tf_msg.child_frame_id = imu_frame_id;
  tf_msg.transform.translation.x = pos.x();
  tf_msg.transform.translation.y = pos.y();
  tf_msg.transform.translation.z = pos.z();
  tf_msg.transform.rotation.x = quat.x();
  tf_msg.transform.rotation.y = quat.y();
  tf_msg.transform.rotation.z = quat.z();
  tf_msg.transform.rotation.w = quat.w();
  tf_broadcaster->sendTransform(tf_msg);
}

void RvizViewer::prepare_clouds(
  const KeyFrame::ConstPtr& frame,
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_imu,
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world) {
  if (!frame || !frame->cloud_imu || frame->cloud_imu->empty()) return;

  cloud_imu->reserve(frame->cloud_imu->size());
  for (const auto& point : *frame->cloud_imu) {
    pcl::PointXYZI point_xyzi;
    point_xyzi.x = point.x;
    point_xyzi.y = point.y;
    point_xyzi.z = point.z;
    point_xyzi.intensity = point.intensity;
    cloud_imu->push_back(point_xyzi);
  }

  if (enable_cloud_world_publish) {
    cloud_world->reserve(cloud_imu->size());
    pcl::transformPointCloud(*cloud_imu, *cloud_world, frame->T_world_imu.matrix().cast<float>());
  }
}

}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::RvizViewer();
}
