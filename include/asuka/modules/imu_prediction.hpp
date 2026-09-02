#pragma once

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

// SO3::Exp(const V3& ang_vel, const scalar& dt), identical to Super-LIO's
// SO3::Exp overload (test/Super-LIO basic/Manifold.cpp:229-241).
inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& ang_vel, double dt) {
  const double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 1e-12) {
    const Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    const Eigen::Matrix3d k = hat(r_axis);
    const double r_ang = ang_vel_norm * dt;
    return Eigen::Matrix3d::Identity() + std::sin(r_ang) * k + (1.0 - std::cos(r_ang)) * k * k;
  }
  return Eigen::Matrix3d::Identity();
}

// High-frequency odometry via pure IMU interpolation, faithful to Super-LIO's
// forward path: ROSWrapper::imuHandler calls ESKF::Predict(imu, state_imu,
// state_robot) (test/Super-LIO ESKF.cpp:136-184) on the forward state
// fw_R_/fw_p_/fw_v_, which is re-anchored inside ESKF::Update() at every scan.
// Here the anchor comes from the frontend KeyFrame instead, and the predicted
// state is published at IMU rate, like the original.
//
// Two inputs the original estimates online cannot be reproduced in a module
// without scan observations, and are taken from config instead:
// - gravity direction is fixed to (0, 0, -gravity); the original estimates
//   the direction from point-to-plane observations and only renormalizes it,
// - imu_scale is the config value; the original estimates it from a static
//   acceleration mean at initialization.
// The robot-frame output (g_odom_robo) is a ROSWrapper concern and is not
// ported; only the IMU-frame state is emitted.
class ImuPrediction : public ExtensionModule {
public:
  ImuPrediction();
  ~ImuPrediction() override;
  void stop() override;

private:
  void on_insert_imu(const ImuData::ConstPtr& imu);
  void on_new_frame(const KeyFrame::ConstPtr& frame);

  struct QueueItem {
    ImuData::ConstPtr imu{nullptr};
    KeyFrame::ConstPtr frame{nullptr};
  };

  // The integration runs exclusively on the processing thread so the ROS
  // subscriber thread and the odometry worker never pay for it. Callbacks
  // only enqueue.
  void processing_thread();
  void process_imu(const ImuData::ConstPtr& imu);
  void process_frame(const KeyFrame::ConstPtr& frame);
  // Integrates one sample against the current forward state; returns false
  // when the sample is stale (dt <= 0) or too far ahead (dt > 0.2), matching
  // the original's Predict return semantics.
  bool try_integrate(const ImuData::ConstPtr& imu);
  void emit_predicted_odometry(double stamp, const Eigen::Matrix3d& R, const Eigen::Vector3d& p,
                               const Eigen::Vector3d& v);

  static constexpr std::size_t queue_capacity = 4096;
  static constexpr std::size_t pending_capacity = 4096;

  double gravity{9.81};
  double imu_scale{1.0};

  int on_insert_imu_id{-1};
  int on_new_frame_id{-1};

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<QueueItem> queue{queue_capacity};
  std::atomic<bool> kill_switch{false};
  std::thread processing_thread_obj;
  std::size_t dropped_items{0};

  // Anchored forward state; only touched by the processing thread.
  Eigen::Matrix3d fw_rot{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d fw_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fw_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_vec{Eigen::Vector3d(0.0, 0.0, -9.81)};
  double forward_time{-1.0};
  ImuData last_imu{};
  bool initialized{false};
  // Samples that arrived ahead of the current anchor (e.g. rosbag playback
  // feeds IMUs at full speed while the frontend lags): held here and
  // integrated in stamp order once the matching keyframe anchor arrives.
  // In real-time operation this stays empty and the original's flow is
  // reproduced exactly.
  boost::circular_buffer<ImuData::ConstPtr> pending_imus{pending_capacity};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka
