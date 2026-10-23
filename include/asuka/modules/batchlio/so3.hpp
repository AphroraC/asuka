#pragma once

#include <cmath>

#include <Eigen/Core>

namespace asuka {
namespace batch {

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template <typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> skew;
  skew << SKEW_SYM_MATRX(v);
  return skew;
}

template <typename T>
Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1>& ang) {
  const T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 0.0000001) {
    const Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> k;
    k << SKEW_SYM_MATRX(r_axis);
    return eye3 + std::sin(ang_norm) * k + (1.0 - std::cos(ang_norm)) * k * k;
  }
  return eye3;
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
  const T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_vel_norm > 0.0000001) {
    const Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> k;
    k << SKEW_SYM_MATRX(r_axis);
    const T r_ang = ang_vel_norm * dt;
    return eye3 + std::sin(r_ang) * k + (1.0 - std::cos(r_ang)) * k * k;
  }
  return eye3;
}

template <typename T>
Eigen::Matrix<T, 3, 3> exp(const T& v1, const T& v2, const T& v3) {
  const T norm = std::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 0.00001) {
    const T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> k;
    k << SKEW_SYM_MATRX(r_ang);
    return eye3 + std::sin(norm) * k + (1.0 - std::cos(norm)) * k * k;
  }
  return eye3;
}

template <typename T>
Eigen::Matrix<T, 3, 1> log(const Eigen::Matrix<T, 3, 3>& R) {
  const T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> k(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * k) : (0.5 * theta / std::sin(theta) * k);
}

template <typename T>
Eigen::Matrix<T, 3, 1> rot_mat_to_euler(const Eigen::Matrix<T, 3, 3>& rot) {
  const T sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  const bool singular = sy < 1e-6;
  T x;
  T y;
  T z;
  if (!singular) {
    x = std::atan2(rot(2, 1), rot(2, 2));
    y = std::atan2(-rot(2, 0), sy);
    z = std::atan2(rot(1, 0), rot(0, 0));
  } else {
    x = std::atan2(-rot(1, 2), rot(1, 1));
    y = std::atan2(-rot(2, 0), sy);
    z = 0;
  }
  return Eigen::Matrix<T, 3, 1>(x, y, z);
}

template <typename T>
Eigen::Matrix3d jacobian_right_inv(Eigen::Vector3d& vec) {
  Eigen::Matrix3d hat_v;
  hat_v << SKEW_SYM_MATRX(vec);
  if (vec.norm() > 1e-6) {
    return Eigen::Matrix3d::Identity() + 0.5 * hat_v +
           (1.0 - vec.norm() * std::cos(vec.norm() / 2) / 2 / std::sin(vec.norm() / 2)) * hat_v * hat_v /
               vec.squaredNorm();
  }
  return Eigen::Matrix3d::Identity();
}

// SO(3) exponential map via Rodrigues' formula. w is a rotation vector (axis * angle).
inline Eigen::Matrix3d so3_exp(const Eigen::Vector3d& w) {
  const double th = w.norm();
  Eigen::Matrix3d k;
  k << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
  if (th < 1e-11) return Eigen::Matrix3d::Identity() + k;
  const Eigen::Matrix3d k_hat = k / th;
  return Eigen::Matrix3d::Identity() + std::sin(th) * k_hat + (1.0 - std::cos(th)) * (k_hat * k_hat);
}

// Intra-window de-skew (Point-LIWO paper eq 3.44-3.47): maps a body point sampled at
// t_j inside a batch window to the window reference time under constant velocity.
inline Eigen::Vector3d deskew_point(const Eigen::Vector3d& p_body, double dt, const Eigen::Vector3d& omg,
                                   const Eigen::Vector3d& vel, const Eigen::Matrix3d& r_i) {
  const Eigen::Matrix3d r_j = so3_exp(omg * dt);
  const Eigen::Vector3d t_j = r_i.transpose() * vel * dt;
  return r_j * p_body + t_j;
}

}  // namespace batch
}  // namespace asuka
