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

// 32-byte slim point, PCL convention (EIGEN_ALIGN16 + PCL_ADD_POINT4D):
// xyz + intensity + per-point time offset. Replaces pcl::PointXYZINormal
// (48 B), whose normal fields were never used anywhere in this project.
struct EIGEN_ALIGN16 PointXYZIOffset {
  PCL_ADD_POINT4D;
  float intensity;
  // Point time offset from the scan time base in milliseconds. Each odometry
  // builds offsets against its own scan time base; the frame-level
  // KeyFrame::stamp then records the first- or last-point stamp, as decided
  // by the specific odometry.
  float offset;

  PointXYZIOffset() {
    x = y = z = 0.0f;
    data[3] = 0.0f;
    intensity = 0.0f;
    offset = 0.0f;
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

static_assert(sizeof(PointXYZIOffset) == 32, "PointXYZIOffset must stay 32 bytes");
using PointT = PointXYZIOffset;
using PointCloudT = pcl::PointCloud<PointT>;
// PointT::offset stores the point time offset from the scan time base in milliseconds.
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

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::PointXYZIOffset,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, offset, offset)
)
// clang-format on
