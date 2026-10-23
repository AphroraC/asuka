#pragma once

// Ported from FAST-LIVO2 include/common_lib.h (+ include/utils/so3_math.h,
// include/utils/types.h). ROS types are replaced by the asuka neutral types
// (asuka::ImuData / asuka::PointT); everything else is kept verbatim so the
// estimator behaves identically to upstream. All content lives inside
// namespace asuka::fastlivo; upstream's file-level namespace imports were
// dropped -- every type is used with explicit qualification or the local
// aliases below (asuka hygiene rule: no using-directives anywhere).

#include <cmath>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Eigen>

#include <sophus/se3.hpp>

#include <asuka/core/types.hpp>

namespace asuka {
namespace fastlivo {

// All names are used with explicit namespace qualification (asuka hygiene
// rule: no using-directives anywhere).

// The upstream code uses the non-template old Sophus `Sophus::SE3d`; the template
// Sophus spells it SE3d. One alias keeps every ported signature verbatim.
#define G_m_s2 (9.81)   // Gravity const in GuangDong/China
#define DIM_STATE (19)  // Dimension of states (Let Dim(SO(3)) = 3)
#define INIT_COV (0.01)
#define SIZE_LARGE (500)
#define SIZE_SMALL (100)
#define DEG2RAD(x) ((x) * (M_PI / 180.0))
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

// ---- so3_math.h ----

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& ang) {
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 0.0000001) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  } else {
    return Eye3;
  }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (ang_vel_norm > 0.0000001) {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;

    K << SKEW_SYM_MATRX(r_axis);

    T r_ang = ang_vel_norm * dt;

    /// Roderigous Tranformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  } else {
    return Eye3;
  }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
  T&& norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 0.00001) {
    T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_ang);

    /// Roderigous Tranformation
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  } else {
    return Eye3;
  }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot) {
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  } else {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

// ---- types.h ----
// (the upstream V2F/V2D/V3D/M3D/V3F/M3F aliases and MD/VD/MF/VF macros are
// gone -- every use site spells Eigen:: types explicitly)

// Upstream named the working point type pcl::PointXYZINormal; asuka's neutral
// PointT (PointXYZITRGB) takes its place. PointT::offset carries the
// per-point time offset in milliseconds exactly like the upstream curvature
// field.
using PointType = PointT;
using PointCloudXYZI = PointCloudT;

struct Pose6D {
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];       // the preintegrated position (global frame) at the Lidar origin
  double rot[9];       // the preintegrated rotation (global frame) at the Lidar origin
};

// ---- common_lib.h ----

enum SLAM_MODE {
  ONLY_LO = 0,
  ONLY_LIO = 1,
  LIVO = 2
};
enum EKF_STATE {
  WAIT = 0,
  VIO = 1,
  LIO = 2,
  LO = 3
};

struct MeasureGroup {
  double vio_time;
  double lio_time;
  // Upstream holds a std::deque<sensor_msgs::Imu::ConstPtr> slice. This slice is a
  // build-once, consume-once sequence (not a std::queue), so a plain std::vector is
  // used; UndistortPcl's push_front becomes an insert at begin().
  std::vector<ImuData::ConstPtr> imu;
  cv::Mat img;
  MeasureGroup() {
    vio_time = 0.0;
    lio_time = 0.0;
  }
};

struct LidarMeasureGroup {
  double lidar_frame_beg_time;
  double lidar_frame_end_time;
  double last_lio_update_time;
  PointCloudT::Ptr lidar;
  PointCloudT::Ptr pcl_proc_cur;
  PointCloudT::Ptr pcl_proc_next;
  // Upstream drains this at every VIO update (LIVO) or lets it grow per scan
  // (ONLY_LIO); the bounded ring drops the oldest measures under overload --
  // the same documented bounded-std::queue divergence as the other frontends.
  boost::circular_buffer<MeasureGroup> measures;
  EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup()
  : measures(50) {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    lio_vio_flg = WAIT;
    lidar.reset(new PointCloudT());
    pcl_proc_cur.reset(new PointCloudT());
    pcl_proc_next.reset(new PointCloudT());
    measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  }
};

typedef struct pointWithVar {
  Eigen::Vector3d point_b;     // point in the lidar body frame
  Eigen::Vector3d point_i;     // point in the imu body frame
  Eigen::Vector3d point_w;     // point in the world frame
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  pointWithVar() {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  }
} pointWithVar;

struct StatesGroup {
  StatesGroup() {
    this->rot_end = Eigen::Matrix3d::Identity();
    this->pos_end = Eigen::Vector3d::Zero();
    this->vel_end = Eigen::Vector3d::Zero();
    this->bias_g = Eigen::Vector3d::Zero();
    this->bias_a = Eigen::Vector3d::Zero();
    this->gravity = Eigen::Vector3d::Zero();
    this->inv_expo_time = 1.0;
    this->cov = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() * INIT_COV;
    this->cov(6, 6) = 0.00001;
    this->cov.block<9, 9>(10, 10) = Eigen::Matrix<double, 9, 9>::Identity() * 0.00001;
  }

  StatesGroup(const StatesGroup& b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  }

  StatesGroup& operator=(const StatesGroup& b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  }

  StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1>& state_add) {
    StatesGroup a;
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);

    a.cov = this->cov;
    return a;
  }

  StatesGroup& operator+=(const Eigen::Matrix<double, DIM_STATE, 1>& state_add) {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
  }

  Eigen::Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b) {
    Eigen::Matrix<double, DIM_STATE, 1> a;
    Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    return a;
  }

  void resetpose() {
    this->rot_end = Eigen::Matrix3d::Identity();
    this->pos_end = Eigen::Vector3d::Zero();
    this->vel_end = Eigen::Vector3d::Zero();
  }

  Eigen::Matrix3d rot_end;                              // the estimated attitude (rotation matrix) at the end lidar point
  Eigen::Vector3d pos_end;                              // the estimated position at the end lidar point (world frame)
  Eigen::Vector3d vel_end;                              // the estimated velocity at the end lidar point (world frame)
  double inv_expo_time;                     // the estimated inverse exposure time (no scale)
  Eigen::Vector3d bias_g;                               // gyroscope bias
  Eigen::Vector3d bias_a;                               // accelerometer bias
  Eigen::Vector3d gravity;                              // the estimated gravity acceleration
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1>& a, const Eigen::Matrix<T, 3, 1>& g, const Eigen::Matrix<T, 3, 1>& v, const Eigen::Matrix<T, 3, 1>& p,
                const Eigen::Matrix<T, 3, 3>& R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++) rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return rot_kp;
}

}  // namespace fastlivo
}  // namespace asuka
