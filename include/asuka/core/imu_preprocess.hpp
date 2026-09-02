#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <asuka/core/types.hpp>

namespace asuka {

class ImuPreprocess {
public:
  virtual ~ImuPreprocess() = default;

  virtual void insert_imu(const ImuData::ConstPtr& imu) { (void)imu; }
  virtual void initialize(double lidar_start_time) { (void)lidar_start_time; }

  virtual bool is_initialized() const { return false; }
  virtual Eigen::Vector3d get_mean_acceleration() const { return Eigen::Vector3d::Zero(); }
  virtual Eigen::Vector3d get_mean_gyroscope() const { return Eigen::Vector3d::Zero(); }

protected:
  ImuPreprocess() = default;
};

}  // namespace asuka
