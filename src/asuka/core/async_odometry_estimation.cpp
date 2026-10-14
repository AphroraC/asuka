#include <asuka/core/async_odometry_estimation.hpp>

#include <chrono>
#include <limits>
#include <thread>

#include <spdlog/spdlog.h>

#include <asuka/core/callbacks.hpp>

namespace asuka {

AsyncOdometryEstimation::AsyncOdometryEstimation(std::shared_ptr<OdometryEstimation> odometry)
: odometry(std::move(odometry)) {
  worker = std::thread(&AsyncOdometryEstimation::loop, this);
}

AsyncOdometryEstimation::~AsyncOdometryEstimation() {
  stop();
}

void AsyncOdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!running || end_of_sequence) return;
    imu_queue.push_back(imu);
  }
  cv.notify_one();
}

void AsyncOdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!running || end_of_sequence) return;
    frame_queue.push_back(std::make_pair(stamp, cloud));
  }
  cv.notify_one();
}

void AsyncOdometryEstimation::insert_image(const ImageData::ConstPtr& image) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (!running || end_of_sequence) return;
    image_queue.push_back(image);
  }
  cv.notify_one();
}

int AsyncOdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(mtx);
  return static_cast<int>(
           imu_queue.size() + frame_queue.size()
           + image_queue.size()
             ) +
         odometry->workload();
}

bool AsyncOdometryEstimation::is_stopped() const {
  return !running.load(std::memory_order_acquire);
}

bool AsyncOdometryEstimation::wait_for(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mtx);
  // Lock order note: this evaluates odometry->workload() under the executor
  // mutex. The worker releases the mutex before calling into the algorithm
  // (events are popped under the lock, processed outside it), so no path
  // holds an algorithm mutex while waiting for the executor mutex.
  return cv.wait_for(lock, timeout, [this] {
    return !running.load() ||
           (imu_queue.empty() && frame_queue.empty() && image_queue.empty() && odometry->workload() == 0);
  });
}

void AsyncOdometryEstimation::stop() {
  end_of_sequence = true;
  // No early return when running is already false: the worker may have
  // stopped itself via the circuit breaker and still needs joining.
  running.exchange(false);
  cv.notify_all();
  if (worker.joinable()) worker.join();
}

PointCloudT::Ptr AsyncOdometryEstimation::save_map() {
  if (running.load()) {
    spdlog::warn("save_map() called while the odometry worker is still running");
  }
  return odometry->save_map();
}

void AsyncOdometryEstimation::loop() {
  while (true) {
    ImuData::ConstPtr imu;
    std::pair<double, PointCloudT::ConstPtr> frame;
    ImageData::ConstPtr image;
    {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock, [this] {
        return !running || !imu_queue.empty() || !frame_queue.empty()
               || !image_queue.empty()
          ;
      });
      if (!running) break;
      const double imu_stamp = imu_queue.empty() ? std::numeric_limits<double>::max() : imu_queue.front()->stamp;
      const double frame_stamp = frame_queue.empty() ? std::numeric_limits<double>::max() : frame_queue.front().first;
      const double image_stamp = image_queue.empty() ? std::numeric_limits<double>::max() : image_queue.front()->stamp;
      if (imu_stamp <= image_stamp && imu_stamp <= frame_stamp) {
        imu = imu_queue.front();
        imu_queue.pop_front();
      } else if (image_stamp <= frame_stamp) {
        image = image_queue.front();
        image_queue.pop_front();
      } else {
        frame = frame_queue.front();
        frame_queue.pop_front();
      }
    }

    // Exception boundary of the pipeline: an algorithm throwing here must
    // not pierce the thread function (std::terminate would take down the
    // whole process, nodelet manager siblings included). Log and skip the
    // event; delivery of the next event proceeds as usual.
    try {
      if (imu) {
        Callbacks::on_insert_imu(imu);
        odometry->insert_imu(imu);
      } else if (image) {
        Callbacks::on_insert_image(image);
        odometry->insert_image(image);
      } else {
        Callbacks::on_insert_frame(frame.first, frame.second);
        odometry->insert_frame(frame.first, frame.second);
      }
      while (odometry->process_once()) {
      }
      consecutive_failures = 0;
    } catch (const std::exception& e) {
      const double stamp = imu ? imu->stamp : (image ? image->stamp : frame.first);
      spdlog::error("[odometry worker] exception in event (stamp={:.3f}): {}; skipping event", stamp, e.what());
      ++consecutive_failures;
    } catch (...) {
      spdlog::error("[odometry worker] unknown exception in event; skipping event");
      ++consecutive_failures;
    }
    if (consecutive_failures >= max_consecutive_failures) {
      spdlog::critical(
        "[odometry worker] {} consecutive event failures; stopping the worker "
        "(no further events are processed; save_map still returns the map "
        "built so far)",
        consecutive_failures);
      running.store(false);
      break;
    }
  }
}

}  // namespace asuka
