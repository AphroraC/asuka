#pragma once

#include <memory>
#include <string>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>

namespace asuka {

class OdometryEstimation {
public:
  virtual ~OdometryEstimation() = default;

  virtual void insert_imu(const ImuData::ConstPtr& imu) { (void)imu; }

  virtual void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
    (void)stamp;
    (void)cloud;
  }

  virtual bool requires_imu() const { return true; }
  virtual int workload() const { return 0; }
  virtual void stop() {}
  virtual void clear_buffers() {}
  virtual PointCloudT::Ptr save_map() { return nullptr; }
  virtual bool process_once() { return false; }

  virtual std::shared_ptr<CloudPreprocess> cloud_preprocess() { return nullptr; }

  static std::shared_ptr<OdometryEstimation> load_module(const std::string& so_name);

protected:
  OdometryEstimation() = default;
};

extern "C" OdometryEstimation* create_odometry_estimation();

}  // namespace asuka
