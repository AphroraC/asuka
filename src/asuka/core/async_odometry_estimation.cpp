#include <asuka/core/async_odometry_estimation.hpp>

#include <chrono>
#include <vector>

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

int AsyncOdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(mtx);
  return static_cast<int>(imu_queue.size() + frame_queue.size()) + odometry->workload();
}

void AsyncOdometryEstimation::wait_idle() const {
  while (true) {
    bool input_pending = false;
    {
      std::lock_guard<std::mutex> lock(mtx);
      input_pending = !imu_queue.empty() || !frame_queue.empty();
    }
    if (!input_pending && !processing && odometry->workload() == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void AsyncOdometryEstimation::stop() {
  end_of_sequence = true;
  if (!running.exchange(false)) return;
  cv.notify_all();
  if (worker.joinable()) worker.join();
}

void AsyncOdometryEstimation::loop() {
  while (true) {
    std::vector<ImuData::ConstPtr> imus;
    std::vector<std::pair<double, PointCloudT::ConstPtr>> frames;
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this] { return !running || !imu_queue.empty() || !frame_queue.empty(); });
    if (!running) break;
    imus.reserve(imu_queue.size());
    imus.insert(imus.end(), imu_queue.begin(), imu_queue.end());
    frames.reserve(frame_queue.size());
    frames.insert(frames.end(), frame_queue.begin(), frame_queue.end());
    imu_queue.clear();
    frame_queue.clear();
    lock.unlock();

    processing = true;
    // Feed IMU samples before frames, matching glim_ros's executor ordering.
    for (const auto& imu : imus) odometry->insert_imu(imu);
    for (const auto& frame : frames) odometry->insert_frame(frame.first, frame.second);

    while (odometry->process_once()) {
    }
    processing = false;
  }
}

}  // namespace asuka
