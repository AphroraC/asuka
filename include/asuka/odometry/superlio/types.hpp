#pragma once

#include <memory>
#include <string>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>

#include <asuka/core/types.hpp>
#include <asuka/odometry/superlio/alias.hpp>
#include <asuka/odometry/superlio/manifold.hpp>

namespace asuka {
namespace superlio {

struct SysState {
  SysState() = default;

  explicit SysState(double time, const So3& R = So3(), const V3& tt = V3::Zero(),
                    const V3& v = V3::Zero(), const V3& bg = V3::Zero(), const V3& ba = V3::Zero())
      : timestamp(time), rot(R), p(tt), v(v), bg(bg), ba(ba) {}

  SysState(double time, const Se3& pose, const V3& vel = V3::Zero())
      : timestamp(time), rot(pose.r), p(pose.t), v(vel) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot;
  V3 p = V3::Zero();
  V3 v = V3::Zero();
  V3 bg = V3::Zero();
  V3 ba = V3::Zero();
};

struct NavState {
  NavState() = default;
  explicit NavState(double time, const So3& R = So3(), const V3& tt = V3::Zero(),
                    const V3& v = V3::Zero())
      : timestamp(time), rot(R), p(tt), v(v) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot = Eye3;
  V3 p = V3::Zero();
  V3 v = V3::Zero();
};

struct DynamicState {
  DynamicState() = default;
  explicit DynamicState(double t, const M3& R, const V3& pp, const V3& v, const V3& w, const V3& a)
      : time(t), rot(R), p(pp), v(v), w(w), a(a) {}

  double time = 0;
  M3 rot = M3::Identity();
  V3 p = V3::Zero();
  V3 v = V3::Zero();
  V3 w = V3::Zero();
  V3 a = V3::Zero();
};

struct PoseT {
  PoseT() = default;
  explicit PoseT(double time, const So3& R = So3(), const V3& tt = V3::Zero())
      : timestamp(time), rot(R), p(tt) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot = Eye3;
  V3 p = V3::Zero();
};

struct ImuSample {
  double secs = 0.0;
  V3 acc = V3::Zero();
  V3 gyr = V3::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LidarData {
  double start_time = 0.0;
  double end_time = 0.0;
  PointCloudT::ConstPtr pc{nullptr};
};

struct MeasureGroup {
  LidarData lidar;
  boost::circular_buffer<ImuSample, Eigen::aligned_allocator<ImuSample>> imu;
};

}  // namespace superlio
}  // namespace asuka
