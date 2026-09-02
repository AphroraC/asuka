#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <common/eigen_types.h>
#include <common/imu.h>
#include <common/measure_group.h>
#include <core/lio/eskf.hpp>
#include <core/lio/imu_processing.hpp>

#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace lightning {

struct ImuMeasureGroup {
  ImuMeasureGroup() = default;
  double lidar_beg_time{0.0};
  double lidar_end_time{0.0};
  PointCloudT::Ptr lidar;
  std::vector<ImuData::ConstPtr> imu;
};

class ImuPreprocess : public asuka::ImuPreprocess {
public:
  ImuPreprocess();
  ~ImuPreprocess() override = default;

  void initialize(double lidar_start_time) override;
  void process(const ImuMeasureGroup& meas, ::lightning::ESKF& kf_state, PointCloudT::Ptr& pcl_out);

  bool is_initialized() const override;
  Eigen::Vector3d get_mean_acceleration() const override;
  Eigen::Vector3d get_mean_gyroscope() const override;

  Eigen::Matrix<double, 12, 12> q() const { return pimu->Q_; }

private:
  std::shared_ptr<::lightning::ImuProcess> pimu{nullptr};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace lightning
}  // namespace asuka
