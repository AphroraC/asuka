#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace asuka {

namespace smallpointlio {

template <class T>
Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1>& v) {
  Eigen::Matrix<T, 3, 3> res;
  res << 0, -v[2], v[1],
         v[2], 0, -v[0],
         -v[1], v[0], 0;
  return res;
}

template <class T>
static inline Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1>& ang) {
  T ang_norm = ang.norm();
  if (ang_norm < std::numeric_limits<T>::epsilon()) {
    return Eigen::Matrix<T, 3, 3>::Identity();
  } else {
    Eigen::Matrix<T, 3, 3> k = hat<T>(ang / ang_norm);
    return Eigen::Matrix<T, 3, 3>::Identity() + std::sin(ang_norm) * k + (1.0 - std::cos(ang_norm)) * k * k;
  }
}

template <class T>
Eigen::Matrix<T, 3, 3> a_matrix(const Eigen::Matrix<T, 3, 1>& v) {
  static_assert(!std::numeric_limits<T>::is_integer);
  Eigen::Matrix<T, 3, 3> res;
  T squared_norm = v.squaredNorm();
  if (squared_norm < std::numeric_limits<T>::epsilon()) {
    res = Eigen::Matrix<T, 3, 3>::Identity();
  } else {
    T norm = std::sqrt(squared_norm);
    Eigen::Matrix<T, 3, 3> hat_v;
    hat_v.noalias() = hat(v);
    res = Eigen::Matrix<T, 3, 3>::Identity() + (1 - std::cos(norm)) / squared_norm * hat_v + (1 - std::sin(norm) / norm) / squared_norm * hat_v * hat_v;
  }
  return res;
}

}  // namespace smallpointlio

}  // namespace asuka
