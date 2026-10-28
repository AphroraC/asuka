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
  bool is_stopped() const;
  void stop();

  bool wait_for(std::chrono::milliseconds timeout);

  PointCloudT::Ptr save_map();

private:
  static constexpr std::size_t imu_queue_capacity = 1000;
  static constexpr std::size_t frame_queue_capacity = 50;
  static constexpr std::size_t image_queue_capacity = 50;
  static constexpr int max_consecutive_failures = 10;

  void loop();

  std::shared_ptr<OdometryEstimation> odometry;
  std::thread worker;
  mutable std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> running{true};
  std::atomic<bool> end_of_sequence{false};
  int consecutive_failures{0};
  boost::circular_buffer<ImuData::ConstPtr> imu_queue{imu_queue_capacity};
  boost::circular_buffer<std::pair<double, PointCloudT::ConstPtr>> frame_queue{frame_queue_capacity};
  boost::circular_buffer<ImageData::ConstPtr> image_queue{image_queue_capacity};
};

}  // namespace asuka
