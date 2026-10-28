#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace asuka {

enum class FrameId { WORLD = 0, IMU = 1, LIDAR = 2, CAMERA = 3 };

struct ImuData {
  using Ptr = std::shared_ptr<ImuData>;
  using ConstPtr = std::shared_ptr<const ImuData>;

  double stamp{0.0};
  Eigen::Vector3d linear_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_vel{Eigen::Vector3d::Zero()};
};

struct ImageData {
  using Ptr = std::shared_ptr<ImageData>;
  using ConstPtr = std::shared_ptr<const ImageData>;

  double stamp{0.0};
  std::shared_ptr<const cv::Mat> image{nullptr};
};

struct EIGEN_ALIGN16 PointXYZITRGB {
  PCL_ADD_POINT4D;
  float intensity;
  PCL_ADD_RGB;
  float curvature;

  PointXYZITRGB() {
    x = y = z = 0.0f;
    data[3] = 0.0f;
    intensity = 0.0f;
    rgb = 0.0f;
    curvature = 0.0f;
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

static_assert(sizeof(PointXYZITRGB) == 32, "PointXYZITRGB must stay 32 bytes");

using PointT = PointXYZITRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointVectorT = std::vector<PointT, Eigen::aligned_allocator<PointT>>;

struct EIGEN_ALIGN16 PointXYZIRGB {
  PCL_ADD_POINT4D;
  float intensity;
  PCL_ADD_RGB;

  PointXYZIRGB() {
    x = y = z = 0.0f;
    data[3] = 0.0f;
    intensity = 0.0f;
    rgb = 0.0f;
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

static_assert(sizeof(PointXYZIRGB) == 32, "PointXYZIRGB must stay 32 bytes");

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

struct EIGEN_ALIGN16 LivoxCustomPoint {
  PCL_ADD_POINT4D;
  float intensity;  // livox reflectivity
  std::uint8_t tag;
  std::uint8_t line;
  std::uint32_t offset_time;  // ns from the scan timebase (livox driver)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class LidarType { LIVOX = 1, ROBOSENSE = 2, LIVOX_CUSTOM = 3 };

inline LidarType parse_lidar_type(const std::string& s) {
  if (s == "LIVOX" || s == "livox") return LidarType::LIVOX;
  if (s == "ROBOSENSE" || s == "robosense") return LidarType::ROBOSENSE;
  if (s == "LIVOX_CUSTOM" || s == "livox_custom") return LidarType::LIVOX_CUSTOM;
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
  PointCloudT::ConstPtr cloud_imu{nullptr};
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

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::LivoxCustomPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (std::uint32_t, offset_time, offset_time)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::RobosensePoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::PointXYZITRGB,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, rgb, rgb)
  (float, curvature, curvature)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(asuka::PointXYZIRGB,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, rgb, rgb)
)
// clang-format on
