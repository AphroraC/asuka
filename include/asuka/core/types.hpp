#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace asuka {

enum class FrameId { WORLD = 0, LIDAR = 1, IMU = 2 };

struct ImuData {
  using Ptr = std::shared_ptr<ImuData>;
  using ConstPtr = std::shared_ptr<const ImuData>;

  double stamp{0.0};
  Eigen::Vector3d linear_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_vel{Eigen::Vector3d::Zero()};
};

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;
// PointT::curvature stores the point time offset from scan start in milliseconds.
using PointVectorT = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

struct EIGEN_ALIGN16 LivoxPoint {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 RobosensePoint {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class LidarType { LIVOX = 1, ROBOSENSE = 2 };

inline LidarType parse_lidar_type(const std::string& s) {
  if (s == "LIVOX" || s == "livox") return LidarType::LIVOX;
  if (s == "ROBOSENSE" || s == "robosense") return LidarType::ROBOSENSE;
  throw std::runtime_error("Unknown lidar_type: " + s);
}

struct KeyFrame {
  using Ptr = std::shared_ptr<KeyFrame>;
  using ConstPtr = std::shared_ptr<const KeyFrame>;

  long id{-1};
  double stamp{0.0};
  FrameId frame_id{FrameId::IMU};
  Eigen::Isometry3d T_world_imu{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d v_world_imu{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 6, 1> imu_bias{Eigen::Matrix<double, 6, 1>::Zero()};
  // Deskewed scan in the IMU frame, freshly allocated per frame and owned by
  // the KeyFrame: observers may hold the frame for as long as they like.
  PointCloudT::Ptr cloud_imu{nullptr};
};

}  // namespace asuka

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::LivoxPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::RobosensePoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
)
// clang-format on
