#pragma once

#include <cstdint>
#include <list>
#include <vector>

#include <Eigen/Core>

#include <ankerl/unordered_dense.h>

namespace smallpointlio {

inline uint64_t hash_position_index(const Eigen::Matrix<uint16_t, 3, 1>& v);

class PointWithDistance {
public:
  Eigen::Vector3f point;
  float distance = 0;

  PointWithDistance(Eigen::Vector3f point, float distance);
  bool operator()(const PointWithDistance& p1, const PointWithDistance& p2) const;
  bool operator<(const PointWithDistance& rhs) const;
};

class IVox {
public:
  explicit IVox(float resolution, size_t capacity);

  bool add_point(const Eigen::Vector3f& point_to_add);
  void get_closest_point(const Eigen::Vector3f& pt, std::vector<Eigen::Vector3f>& closest_pt, size_t max_num = 5);

  [[nodiscard]] Eigen::Matrix<uint16_t, 3, 1> get_position_index(const Eigen::Vector3f& pt) const;

private:
  ankerl::unordered_dense::map<uint64_t, std::list<Eigen::Vector3f>::iterator> grids_map;
  float inv_resolution;
  size_t capacity;
  std::list<Eigen::Vector3f> grids_cache_;
  std::vector<PointWithDistance> candidates;
};

inline __attribute__((always_inline)) uint64_t hash_position_index(const Eigen::Matrix<uint16_t, 3, 1>& v) {
  return (static_cast<uint64_t>(v[0]) << 32) |
         (static_cast<uint64_t>(v[1]) << 16) |
         static_cast<uint64_t>(v[2]);
}

inline __attribute__((always_inline)) PointWithDistance::PointWithDistance(Eigen::Vector3f point, float distance)
  : point(std::move(point)),
    distance(distance) {}

inline __attribute__((always_inline)) bool PointWithDistance::operator()(const PointWithDistance& p1, const PointWithDistance& p2) const {
  return p1.distance < p2.distance;
}

inline __attribute__((always_inline)) bool PointWithDistance::operator<(const PointWithDistance& rhs) const {
  return distance < rhs.distance;
}

inline __attribute__((always_inline)) IVox::IVox(float resolution, size_t capacity)
  : inv_resolution(1 / resolution),
    capacity(capacity) {}

inline __attribute__((always_inline)) void IVox::get_closest_point(const Eigen::Vector3f& pt, std::vector<Eigen::Vector3f>& closest_pt, size_t max_num) {
  closest_pt.clear();
  Eigen::Matrix<uint16_t, 3, 1> key = get_position_index(pt);
  uint64_t hash_key = hash_position_index(key);
  ankerl::unordered_dense::map<uint64_t, std::list<Eigen::Vector3f>::iterator>::iterator iter;

  auto try_add = [&](uint64_t hk) {
    auto it = grids_map.find(hk);
    if (it != grids_map.end()) {
      closest_pt.push_back(*it->second);
    }
  };

  try_add(hash_key);
  try_add((hash_key & 0xFFFFFFFFFFFF0000) | ((hash_key + 1L) & 0x000000000000FFFF));
  try_add((hash_key & 0xFFFFFFFFFFFF0000) | ((hash_key - 1L) & 0x000000000000FFFF));
  try_add((hash_key & 0xFFFFFFFF0000FFFF) | ((hash_key + (1L << 16)) & 0x00000000FFFF0000));
  try_add((hash_key & 0xFFFFFFFF0000FFFF) | ((hash_key - (1L << 16)) & 0x00000000FFFF0000));
  try_add((hash_key & 0xFFFF0000FFFFFFFF) | ((hash_key + (1L << 32)) & 0x0000FFFF00000000));
  try_add((hash_key & 0xFFFF0000FFFFFFFF) | ((hash_key - (1L << 32)) & 0x0000FFFF00000000));

  if (closest_pt.size() > max_num) [[likely]] {
    candidates.clear();
    for (auto& point : closest_pt) {
      candidates.emplace_back(point, (point - pt).squaredNorm());
    }
    std::nth_element(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(max_num) - 1, candidates.end());
    closest_pt.clear();
    for (size_t i = 0; i < max_num; ++i) {
      closest_pt.push_back(candidates[i].point);
    }
  }
}

inline __attribute__((always_inline)) bool IVox::add_point(const Eigen::Vector3f& point) {
  Eigen::Matrix<uint16_t, 3, 1> key = get_position_index(point);
  auto hash_key = hash_position_index(key);
  auto iter = grids_map.find(hash_key);
  if (iter != grids_map.end()) {
    grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
    grids_map[hash_key] = grids_cache_.begin();
    return false;
  } else {
    grids_cache_.push_front(point);
    grids_map.emplace(hash_key, grids_cache_.begin());
    if (grids_map.size() >= capacity) {
      grids_map.erase(hash_position_index(get_position_index(grids_cache_.back())));
      grids_cache_.pop_back();
    }
    return true;
  }
}

[[nodiscard]] inline __attribute__((always_inline)) Eigen::Matrix<uint16_t, 3, 1> IVox::get_position_index(const Eigen::Vector3f& pt) const {
  return (pt * inv_resolution).array().floor().cast<uint16_t>();
}

}  // namespace smallpointlio
