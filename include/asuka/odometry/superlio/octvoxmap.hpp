#pragma once

#include <array>
#include <cstring>
#include <list>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <tsl/robin_map.h>

#include <asuka/odometry/superlio/alias.hpp>
#include <asuka/odometry/superlio/hknn_tables.hpp>

namespace asuka {
namespace superlio {

template <int K, typename Point>
class KnnHeap {
public:
  KnnHeap() : count(0), worst(0), max_dist2(0.0f) {
    memset(dist2, 0, sizeof(dist2));
  }

  void reset() {
    count = 0;
    worst = 0;
    max_dist2 = 0.0f;
    memset(dist2, 0, sizeof(dist2));
  }

  std::uint8_t count;
  std::uint8_t worst;
  float max_dist2;
  float dist2[K];
  std::array<Point, K> points;

  inline void try_insert(float d2, const Point& pt) {
    const bool not_full = (count < K);
    const bool should_insert = not_full || (d2 < max_dist2);

    if (should_insert) {
      const std::uint8_t insert_idx = not_full ? count : worst;

      dist2[insert_idx] = d2;
      points[insert_idx] = pt;

      if (not_full) {
        count++;
        if (d2 > max_dist2) {
          max_dist2 = d2;
          worst = insert_idx;
        }
      } else {
        update_worst_unrolled();
      }
    }
  }

  inline float max_dist2_value() const { return max_dist2; }

private:
  inline void update_worst_unrolled() {
    float d0 = dist2[0], d1 = dist2[1], d2 = dist2[2], d3 = dist2[3], d4 = dist2[4];

    std::uint8_t idx01 = d0 > d1 ? 0 : 1;
    float max01 = d0 > d1 ? d0 : d1;

    std::uint8_t idx23 = d2 > d3 ? 2 : 3;
    float max23 = d2 > d3 ? d2 : d3;

    std::uint8_t idx0123 = max01 > max23 ? idx01 : idx23;
    float max0123 = max01 > max23 ? max01 : max23;

    worst = max0123 > d4 ? idx0123 : 4;
    max_dist2 = max0123 > d4 ? max0123 : d4;
  }
};

template <typename Point>
class OctVox {
public:
  OctVox(const Point& pt, std::uint8_t local_idx) {
    counts.fill(uninit_mask);
    points[local_idx] = pt;
    counts[local_idx] = 1;
  }

  ~OctVox() {}

  void add_point(const Point& pt, std::uint8_t local_idx) {
    std::uint8_t& cnt = counts[local_idx];
    Point& stored_point = points[local_idx];
    if (cnt == uninit_mask) {
      stored_point = pt;
      cnt = 1;
      return;
    }

    if (cnt >= max_points_per_subvoxel) return;
    if ((pt - stored_point).squaredNorm() > distance_threshold_sq) return;

    stored_point = (stored_point * cnt + pt) / (cnt + 1);
    ++cnt;
  }

  bool get_point(std::uint8_t local_idx, Point& pt) const {
    if (counts[local_idx] == uninit_mask) return false;
    pt = points[local_idx];
    return true;
  }

  static constexpr std::uint8_t uninit_mask = 0x00;
  static constexpr std::uint8_t max_points_per_subvoxel = 20;
  static constexpr double distance_threshold_sq = 0.1 * 0.1;

  std::array<std::uint8_t, 8> counts;
  std::array<Point, 8> points;
};

template <typename Point, typename Scalar>
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using Key = Eigen::Vector3i;
  using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;
  using KnnHeapType = KnnHeap<5, Point>;
  using OctVoxType = OctVox<Point>;

  struct Options {
    float resolution = 0.5f;
    std::size_t capacity = 1000000;

    Options(float r, std::size_t c) {
      resolution = r;
      capacity = c;
    }
  };

  OctVoxMap() {
    flat_search_ptrs.reserve(flat_search_order_offsets.size());
    for (std::size_t i = 0; i < flat_search_order_offsets.size(); i++) {
      std::uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs.push_back(const_cast<std::uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max = flat_search_order_offsets.size() - 1;
  }

  ~OctVoxMap() {
    grids.clear();
    data.clear();
  }

  OctVoxMap(Options options) {
    set_options(options);
    flat_search_ptrs.reserve(flat_search_order_offsets.size());
    for (std::size_t i = 0; i < flat_search_order_offsets.size(); i++) {
      std::uint16_t start = flat_search_order_offsets[i];
      flat_search_ptrs.push_back(const_cast<std::uint8_t*>(flat_search_order.data() + start));
    }
    group_idx_max = flat_search_order_offsets.size() - 1;
  }

  void set_options(const Options& options) {
    resolution = options.resolution;
    capacity = options.capacity;
    inv_resolution = 1.0f / resolution;
    sub_resolution = resolution / 2.0f;
    sub_inv_resolution = 1.0f / sub_resolution;
  }

  void insert(const Points& cloud_world);
  void get_top_k(const Point& point, KnnHeapType& top_k) const;

  void reset_max_group() {
    group_idx_max = flat_search_order_offsets.size() - 1;
  }

  void decrease_max_group() {
    if (group_idx_max > 4) group_idx_max--;
  }

  void clear() {
    grids.clear();
    data.clear();
  }

private:
  float resolution = 0.5f;
  float inv_resolution = 1.0f;
  float sub_resolution = 0.25f;
  float sub_inv_resolution = 4.0f;
  std::size_t capacity = 1000000;

  bool reset_map = false;
  int reset_map_count = 0;

  struct HashVec {
    std::size_t operator()(const Key& v) const {
      std::size_t h = static_cast<std::size_t>(v[0]);
      h ^= v[1] * 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= v[2] * 0x85ebca6b + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DataList = std::list<std::pair<Key, OctVoxType>>;
  using DataIter = typename DataList::iterator;

  DataList data;
  tsl::robin_map<Key, DataIter, HashVec> grids;

  std::vector<std::uint8_t*> flat_search_ptrs;
  int group_idx_max;
};

template <typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::insert(const Points& cloud_world) {
  if (reset_map) {
    reset_map_count--;
    if (reset_map_count > 0) {
      return;
    }
    reset_map = false;
  }

  for (auto& pt : cloud_world) {
    Key fine_key = (pt * sub_inv_resolution).array().floor().template cast<int>();
    Key key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    std::uint8_t dx = fine_key[0] & 1;
    std::uint8_t dy = fine_key[1] & 1;
    std::uint8_t dz = fine_key[2] & 1;
    std::uint8_t local_idx = (dz << 2) | (dy << 1) | dx;

    auto iter = grids.find(key);
    if (iter == grids.end()) {
      data.emplace_front(std::piecewise_construct, std::forward_as_tuple(key),
                         std::forward_as_tuple(pt, local_idx));
      grids.insert(std::make_pair(key, data.begin()));

      if (data.size() >= capacity) {
        grids.erase(data.back().first);
        data.pop_back();
      }
    } else {
      iter->second->second.add_point(pt, local_idx);
      data.splice(data.begin(), data, iter->second);
    }
  }
}

template <typename Point, typename Scalar>
void OctVoxMap<Point, Scalar>::get_top_k(const Point& point, KnnHeapType& top_k) const {
  const Key fine_key = (point * sub_inv_resolution).array().floor().template cast<int>();
  Key key;
  key[0] = fine_key[0] >> 1;
  key[1] = fine_key[1] >> 1;
  key[2] = fine_key[2] >> 1;

  const int dx = fine_key[0] & 1;
  const int dy = fine_key[1] & 1;
  const int dz = fine_key[2] & 1;
  const int local_idx = (dz << 2) | (dy << 1) | dx;
  const Key mirror_axis = Key(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));

  const int pre_voxel_ptr_size = 8;
  OctVoxType* top_voxels_2_search[pre_voxel_ptr_size];
  std::fill_n(top_voxels_2_search, pre_voxel_ptr_size, nullptr);

  for (std::uint8_t i = 0; i < pre_voxel_ptr_size; ++i) {
    Key delta_key = mirror_axis.cwiseProduct(hknn_neighbor_voxel[i]);
    Key n_key = key + delta_key;
    if (auto iter = grids.find(n_key); iter != grids.end()) {
      top_voxels_2_search[i] = &iter->second->second;
    }
  }

  Point sub_point;

  for (int group_idx = 0; group_idx < group_idx_max; ++group_idx) {
    const std::uint8_t* group_it = flat_search_ptrs[group_idx];
    const std::uint8_t* group_end = flat_search_ptrs[group_idx + 1];

    while (group_it < group_end) {
      const std::uint8_t neighbor_idx = *group_it++;
      std::uint8_t data_size = *group_it++;

      if (neighbor_idx < pre_voxel_ptr_size) {
        OctVoxType* voxel_ptr = top_voxels_2_search[neighbor_idx];
        if (voxel_ptr) {
          while (data_size--) {
            std::uint8_t local_i = (*group_it++) ^ local_idx;
            if (voxel_ptr->get_point(local_i, sub_point)) {
              const float d2 = (sub_point - point).squaredNorm();
              top_k.try_insert(d2, sub_point);
            }
          }
        } else {
          group_it += data_size;
        }
        continue;
      }

      Key delta_key = mirror_axis.cwiseProduct(hknn_neighbor_voxel[neighbor_idx]);
      const Key n_key = key + delta_key;

      if (auto iter = grids.find(n_key); iter != grids.end()) {
        OctVoxType* voxel_ptr = &iter->second->second;
        while (data_size--) {
          const std::uint8_t local_i = (*group_it++) ^ local_idx;
          if (voxel_ptr->get_point(local_i, sub_point)) {
            float d2 = (sub_point - point).squaredNorm();
            top_k.try_insert(d2, sub_point);
          }
        }
      } else {
        group_it += data_size;
      }
    }

    if (top_k.count == 5) {
      if (top_k.max_dist2 < orders_min_dis2[group_idx]) {
        break;
      }
    }
  }
}

}  // namespace superlio
}  // namespace asuka
