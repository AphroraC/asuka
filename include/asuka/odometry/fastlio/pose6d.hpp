#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ikfom {

/**
 * @brief Lightweight 6-DOF pose used internally by the IMU forward propagator.
 *
 * This is the plain-C++ replacement for fastlio's `fast_lio::Pose6D` ROS message, used
 * to cache the preintegrated IMU state at every IMU sample within a LiDAR scan so that
 * the backward propagation can undistort the points.
 */
struct Pose6D {
  double offset_time{0.0};            // Offset time w.r.t. the first LiDAR point (s)
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};   // World-frame acceleration at IMU origin
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};   // Body-frame unbiased angular velocity
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};   // World-frame velocity
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};   // World-frame position
  Eigen::Matrix3d rot{Eigen::Matrix3d::Identity()};  // World-frame rotation
};

/**
 * @brief Helper to build a Pose6D from individual components (matches fastlio's
 *        `set_pose6d` helper).
 */
inline Pose6D set_pose6d(double t,
                         const Eigen::Vector3d& a,
                         const Eigen::Vector3d& g,
                         const Eigen::Vector3d& v,
                         const Eigen::Vector3d& p,
                         const Eigen::Matrix3d& R) {
  Pose6D pose;
  pose.offset_time = t;
  pose.acc = a;
  pose.gyro = g;
  pose.vel = v;
  pose.pos = p;
  pose.rot = R;
  return pose;
}

}  // namespace ikfom
