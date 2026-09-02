#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <boost/circular_buffer.hpp>

#include <asuka/core/odometry_estimation.hpp>

namespace asuka {

class AsyncOdometryEstimation {
public:
  explicit AsyncOdometryEstimation(std::shared_ptr<OdometryEstimation> odometry);
  ~AsyncOdometryEstimation();

  void insert_imu(const ImuData::ConstPtr& imu);
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud);

  int workload() const;
  void wait_idle() const;
  void stop();

private:
  static constexpr std::size_t imu_queue_capacity = 1000;
  static constexpr std::size_t frame_queue_capacity = 50;

  void loop();

  std::shared_ptr<OdometryEstimation> odometry;
  std::thread worker;
  mutable std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> running{true};
  std::atomic<bool> end_of_sequence{false};
  std::atomic<bool> processing{false};
  boost::circular_buffer<ImuData::ConstPtr> imu_queue{imu_queue_capacity};
  boost::circular_buffer<std::pair<double, PointCloudT::ConstPtr>> frame_queue{frame_queue_capacity};
};

}  // namespace asuka
