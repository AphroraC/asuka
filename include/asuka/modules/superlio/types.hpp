#pragma once

#include <memory>
#include <string>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>

#include <asuka/core/types.hpp>
#include <asuka/modules/superlio/alias.hpp>
#include <asuka/modules/superlio/manifold.hpp>

namespace asuka {
namespace superlio {

struct SysState {
  SysState() = default;

  explicit SysState(double time, const So3& R = So3(), const Eigen::Matrix<float, 3, 1>& tt = Eigen::Matrix<float, 3, 1>::Zero(),
                    const Eigen::Matrix<float, 3, 1>& v = Eigen::Matrix<float, 3, 1>::Zero(), const Eigen::Matrix<float, 3, 1>& bg = Eigen::Matrix<float, 3, 1>::Zero(), const Eigen::Matrix<float, 3, 1>& ba = Eigen::Matrix<float, 3, 1>::Zero())
      : timestamp(time), rot(R), p(tt), v(v), bg(bg), ba(ba) {}

  SysState(double time, const Se3& pose, const Eigen::Matrix<float, 3, 1>& vel = Eigen::Matrix<float, 3, 1>::Zero())
      : timestamp(time), rot(pose.r), p(pose.t), v(vel) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot;
  Eigen::Matrix<float, 3, 1> p = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> v = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> bg = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> ba = Eigen::Matrix<float, 3, 1>::Zero();
};

struct NavState {
  NavState() = default;
  explicit NavState(double time, const So3& R = So3(), const Eigen::Matrix<float, 3, 1>& tt = Eigen::Matrix<float, 3, 1>::Zero(),
                    const Eigen::Matrix<float, 3, 1>& v = Eigen::Matrix<float, 3, 1>::Zero())
      : timestamp(time), rot(R), p(tt), v(v) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot = Eye3;
  Eigen::Matrix<float, 3, 1> p = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> v = Eigen::Matrix<float, 3, 1>::Zero();
};

struct DynamicState {
  DynamicState() = default;
  explicit DynamicState(double t, const Eigen::Matrix<float, 3, 3>& R, const Eigen::Matrix<float, 3, 1>& pp, const Eigen::Matrix<float, 3, 1>& v, const Eigen::Matrix<float, 3, 1>& w, const Eigen::Matrix<float, 3, 1>& a)
      : time(t), rot(R), p(pp), v(v), w(w), a(a) {}

  double time = 0;
  Eigen::Matrix<float, 3, 3> rot = Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, 3, 1> p = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> v = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> w = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> a = Eigen::Matrix<float, 3, 1>::Zero();
};

struct PoseT {
  PoseT() = default;
  explicit PoseT(double time, const So3& R = So3(), const Eigen::Matrix<float, 3, 1>& tt = Eigen::Matrix<float, 3, 1>::Zero())
      : timestamp(time), rot(R), p(tt) {}

  Se3 get_se3() const { return Se3(rot, p); }

  double timestamp = 0;
  So3 rot = Eye3;
  Eigen::Matrix<float, 3, 1> p = Eigen::Matrix<float, 3, 1>::Zero();
};

struct ImuSample {
  double secs = 0.0;
  Eigen::Matrix<float, 3, 1> acc = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> gyr = Eigen::Matrix<float, 3, 1>::Zero();
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
