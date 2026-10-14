#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <boost/circular_buffer.hpp>

#include <asuka/core/odometry_estimation.hpp>

namespace asuka {

class AsyncOdometryEstimation {
public:
  explicit AsyncOdometryEstimation(std::shared_ptr<OdometryEstimation> odometry);
  ~AsyncOdometryEstimation();

  void insert_imu(const ImuData::ConstPtr& imu);
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud);
  void insert_image(const ImageData::ConstPtr& image);

  int workload() const;
  // True once stop() ran or the worker tripped the consecutive-failure
  // circuit breaker: the backlog can no longer drain, so waiters (the entry
  // points' wait(auto_quit)) must not keep waiting for workload() == 0.
  bool is_stopped() const;
  void stop();

  /// Blocks up to @p timeout for the input queues AND the algorithm's
  /// pending work (odometry->workload()) to drain to zero, or for the
  /// worker to stop (stop() / circuit breaker). Returns true when either
  /// happened, false on timeout. The condition-variable wait replaces the
  /// entry points' 100 Hz workload() polling: the drained state is detected
  /// exactly when the last event is consumed, and a self-stopped worker
  /// wakes waiters immediately instead of after the next poll. Bounded so
  /// external shutdowns (ros::ok()) stay responsive -- an unbounded cv wait
  /// could never be interrupted by Ctrl-C.
  bool wait_for(std::chrono::milliseconds timeout);

  PointCloudT::Ptr save_map();

private:
  static constexpr std::size_t imu_queue_capacity = 1000;
  static constexpr std::size_t frame_queue_capacity = 50;
  static constexpr std::size_t image_queue_capacity = 50;
  // Consecutive failed events after which the worker stops itself: a run of
  // failures means the algorithm is systematically broken (state machine
  // desync, config mismatch), and skipping forever would silently idle the
  // pipeline. One failed event is still just skipped.
  static constexpr int max_consecutive_failures = 10;

  void loop();

  std::shared_ptr<OdometryEstimation> odometry;
  std::thread worker;
  mutable std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> running{true};
  std::atomic<bool> end_of_sequence{false};
  // Worker-thread only: consecutive failed events; any success resets it.
  int consecutive_failures{0};
  boost::circular_buffer<ImuData::ConstPtr> imu_queue{imu_queue_capacity};
  boost::circular_buffer<std::pair<double, PointCloudT::ConstPtr>> frame_queue{frame_queue_capacity};
  boost::circular_buffer<ImageData::ConstPtr> image_queue{image_queue_capacity};
};

}  // namespace asuka
