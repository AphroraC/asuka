#pragma once
// Ported from sr_livo include/lioOptimization.h: the measurement/window data
// structures shared between the odometry class and the vision pipeline.
// The ROS-facing members (publishers, tf, topic strings) are not migrated.

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/types.hpp>
#include <asuka/odometry/srlivo/cloud_map.hpp>

namespace asuka {
namespace srlivo {

class state;

struct Measurements {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<ImuData::ConstPtr> imu_measurements;

  std::pair<double, double> time_sweep;
  std::vector<point3D> lidar_points;

  double time_image = 0.0;
  ImageData::ConstPtr image;

  bool rendering = false;
};

class cloudFrame {
public:
  double time_sweep_begin, time_sweep_end;
  double time_frame_begin, time_frame_end;

  int id;      // the index in all_cloud_frame
  int sub_id;  // the index of segment
  int frame_id;

  state *p_state;

  std::vector<point3D> point_frame;

  double offset_begin;
  double offset_end;
  double dt_offset;

  bool success;

  cloudFrame(std::vector<point3D> &point_frame_, state *p_state_);

  cloudFrame(cloudFrame *p_cloud_frame);

  void release();

  // The rendered image of the window. The Mat headers below share the pixel
  // buffer of the ImageData payload, which is held through image_data so the
  // pixels stay alive for the frame lifetime; nothing is ever cloned.
  ImageData::ConstPtr image_data;
  cv::Mat rgb_image;
  cv::Mat gray_image;

  int image_cols;
  int image_rows;

  bool if2dPointsAvailable(const double &u, const double &v, const double &scale = 1.0, double fov_mar = -1.0);

  bool getRgb(const double &u, const double &v, int &r, int &g, int &b);

  Eigen::Vector3d getRgb(double &u, double &v, int layer, Eigen::Vector3d *rgb_dx = nullptr, Eigen::Vector3d *rgb_dy = nullptr);

  bool project3dTo2d(const pcl::PointXYZI &point_in, double &u, double &v, const double &scale);

  bool project3dPointInThisImage(const pcl::PointXYZI &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale);

  bool project3dPointInThisImage(const Eigen::Vector3d &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale);

  void refreshPoseForProjection();
};

struct Neighborhood {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  Eigen::Vector3d normal = Eigen::Vector3d::Zero();

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  double a2D = 1.0;  // Planarity coefficient
};

struct ResidualBlock {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d point_closest;

  Eigen::Vector3d pt_imu;

  Eigen::Vector3d normal;

  double alpha_time;

  double weight;
};

class estimationSummary {
public:
  int sample_size = 0;               // The number of points sampled
  int number_of_residuals = 0;       // The number of keypoints used for ICP registration
  int robust_level = 0;
  double distance_correction = 0.0;  // The correction between the last frame end and the new frame beginning
  double relative_distance = 0.0;    // The distance between the beginning of the new frame and the end
  double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end
  double ego_orientation = 0.0;      // The angular distance between the beginning and the end of the frame
  bool success = true;               // Whether the registration was a success

  std::string error_message;

  estimationSummary();

  void release();
};

struct optimizeSummary {
  bool success = false;  // Whether the registration succeeded
  int num_residuals_used = 0;
  std::string error_log;
};

}  // namespace srlivo
}  // namespace asuka
