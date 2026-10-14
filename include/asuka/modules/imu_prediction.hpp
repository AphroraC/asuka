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
#include <asuka/utility/joining_thread.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

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

  void processing_thread();
  void process_imu(const ImuData::ConstPtr& imu);
  void process_frame(const KeyFrame::ConstPtr& frame);
  bool try_integrate(const ImuData::ConstPtr& imu);
  void
  emit_predicted_odometry(double stamp, const Eigen::Matrix3d& R, const Eigen::Vector3d& p, const Eigen::Vector3d& v);

  static constexpr std::size_t queue_capacity = 4096;
  static constexpr std::size_t pending_capacity = 4096;

  double gravity{9.81};

  // RAII subscriptions: destruction removes the callbacks. Declared before
  // the processing thread so destruction joins the thread first, then
  // unsubscribes (no in-flight emissions at unsubscribe time).
  CallbackSubscription<void(const ImuData::ConstPtr&)> on_insert_imu_sub;
  CallbackSubscription<void(const KeyFrame::ConstPtr&)> on_new_frame_sub;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<QueueItem> queue{queue_capacity};
  std::atomic<bool> kill_switch{false};
  JoiningThread processing_thread_obj;
  std::size_t dropped_items{0};

  Eigen::Matrix3d fw_rot{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d fw_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fw_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_vec{Eigen::Vector3d(0.0, 0.0, -9.81)};
  double forward_time{-1.0};
  ImuData last_imu{};
  bool initialized{false};
  boost::circular_buffer<ImuData::ConstPtr> pending_imus{pending_capacity};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka
