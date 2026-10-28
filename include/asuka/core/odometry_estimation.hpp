#pragma once

#include <memory>
#include <string>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>

namespace asuka {

class OdometryEstimation {
public:
  virtual ~OdometryEstimation() = default;

  virtual void insert_imu(const ImuData::ConstPtr& imu) = 0;
  virtual void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) = 0;

  virtual void insert_image(const ImageData::ConstPtr& image) = 0;

  virtual int workload() const = 0;
  virtual void stop() = 0;
  virtual PointCloudT::Ptr save_map() = 0;
  virtual bool process_once() = 0;

  virtual std::shared_ptr<CloudPreprocess> cloud_preprocess() = 0;

  static std::shared_ptr<OdometryEstimation> load_module(const std::string& so_name);

  static std::shared_ptr<OdometryEstimation> load_configured();

protected:
  OdometryEstimation() = default;
};

extern "C" OdometryEstimation* create_odometry_estimation();

}  // namespace asuka
