#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief Skew-symmetric matrix macro for raw 3D vectors (used in fastlio-style h_share_model).
 */
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

namespace ikfom {

/**
 * @brief SO(3) math utilities for rotation operations.
 *
 * Provides skew-symmetric matrix, matrix exponential (Rodrigues'),
 * and logarithm for SO(3).
 */

/**
 * @brief Compute the skew-symmetric matrix of a 3D vector.
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> skewSymmetric(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> skew;
  skew << static_cast<T>(0.0), -v(2), v(1),
          v(2), static_cast<T>(0.0), -v(0),
         -v(1), v(0), static_cast<T>(0.0);
  return skew;
}

/**
 * @brief Matrix exponential on so(3) -> SO(3) (Rodrigues' formula).
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> expSo3(const Eigen::Matrix<T, 3, 1>& ang) {
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > static_cast<T>(1e-7)) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K = skewSymmetric(r_axis);
    return eye3 + std::sin(ang_norm) * K +
           (static_cast<T>(1.0) - std::cos(ang_norm)) * K * K;
  }
  return eye3;
}

/**
 * @brief Matrix exponential on so(3) with time scaling.
 */
template <typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> expSo3(const Eigen::Matrix<T, 3, 1>& ang_vel,
                                     const Ts& dt) {
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_vel_norm > static_cast<T>(1e-7)) {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K = skewSymmetric(r_axis);
    T r_ang = ang_vel_norm * dt;
    return eye3 + std::sin(r_ang) * K +
           (static_cast<T>(1.0) - std::cos(r_ang)) * K * K;
  }
  return eye3;
}

/**
 * @brief Logarithm of a rotation matrix (SO(3) -> so(3)).
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> logSo3(const Eigen::Matrix<T, 3, 3>& R) {
  T theta = (R.trace() > static_cast<T>(3.0 - 1e-6))
                ? static_cast<T>(0.0)
                : std::acos(static_cast<T>(0.5) * (R.trace() - static_cast<T>(1.0)));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2),
                           R(0, 2) - R(2, 0),
                           R(1, 0) - R(0, 1));
  return (std::abs(theta) < static_cast<T>(0.001))
             ? (static_cast<T>(0.5) * K)
             : (static_cast<T>(0.5) * theta / std::sin(theta) * K);
}

/**
 * @brief Convert rotation matrix to Euler angles (roll, pitch, yaw).
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> rotmToEuler(const Eigen::Matrix<T, 3, 3>& rot) {
  T sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < static_cast<T>(1e-6);
  T x, y, z;
  if (!singular) {
    x = std::atan2(rot(2, 1), rot(2, 2));
    y = std::atan2(-rot(2, 0), sy);
    z = std::atan2(rot(1, 0), rot(0, 0));
  } else {
    x = std::atan2(-rot(1, 2), rot(1, 1));
    y = std::atan2(-rot(2, 0), sy);
    z = static_cast<T>(0.0);
  }
  return Eigen::Matrix<T, 3, 1>(x, y, z);
}

} // namespace ikfom
