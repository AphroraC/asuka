#pragma once

#include <boost/circular_buffer.hpp>
#include <memory>

#include <Eigen/Core>

#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/superlio/alias.hpp>
#include <asuka/odometry/superlio/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace superlio {

class ImuPreprocess : public asuka::ImuPreprocess {
public:
  ImuPreprocess();
  ~ImuPreprocess() override = default;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void initialize(double lidar_start_time) override { (void)lidar_start_time; }
  bool is_initialized() const override { return false; }
  Eigen::Vector3d get_mean_acceleration() const override { return mean_acc.cast<double>(); }
  Eigen::Vector3d get_mean_gyroscope() const override { return mean_gyro.cast<double>(); }

  void accumulate(const ImuSample& imu);
  void reset_accumulators();

  boost::circular_buffer<ImuSample, Eigen::aligned_allocator<ImuSample>> imu_buffer;
  double last_timestamp{-1.0};

  int imu_count{0};
  V3 mean_gyro{V3::Zero()};
  V3 mean_acc{V3::Zero()};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace superlio
}  // namespace asuka
