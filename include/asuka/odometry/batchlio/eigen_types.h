//
// Created by xiang on 2021/7/16.
//

#ifndef FASTER_LIO_EIGEN_TYPES_H
#define FASTER_LIO_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fasterlio {

/// less of vector
template <int N>
struct less_vec {
  inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

/// hash of vector
template <int N>
struct hash_vec {
  inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

/// implementation
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) && (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/// vec 2 hash
/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
  return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
}

/// vec 3 hash
template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
  return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
}

// constexpr auto less_vec2i = [](const Vec2i& v1, const Vec2i& v2) {
//     return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
// };

}  // namespace fasterlio

#endif
