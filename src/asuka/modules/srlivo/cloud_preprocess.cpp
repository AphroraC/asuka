#include <asuka/modules/srlivo/cloud_preprocess.hpp>

#include <asuka/modules/srlivo/cloud_map.hpp>
#include <asuka/modules/srlivo/utility.hpp>

#include <algorithm>
#include <cmath>
#include <cassert>

#include <asuka/utility/logging.hpp>

namespace asuka {
namespace srlivo {

namespace {

// upstream IS_VALID: true when the coordinate is garbage (|a| > 1e8)
inline bool is_invalid(double a) { return std::abs(a) > 1e8; }

inline void push_point(PointCloudT::Ptr &out, const Eigen::Vector3d &raw, double offset_ms, float intensity) {
  PointT pt;
  pt.x = static_cast<float>(raw.x());
  pt.y = static_cast<float>(raw.y());
  pt.z = static_cast<float>(raw.z());
  pt.intensity = intensity;
  pt.curvature = static_cast<float>(offset_ms);
  pt.rgb = 0.0f;
  out->points.push_back(pt);
}

}  // namespace

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr &raw) {
  assert(stamp > last_time_lidar);
  last_time_lidar = stamp;

  PointCloudT::Ptr out(new PointCloudT());
  const int point_size = static_cast<int>(raw->size());
  if (point_size < 2) return out;

  // first pass: r3live point selection (raw adjacent-pair dedup, no
  // point_filter_num/blind here -- they live in the second pass)
  std::vector<point3D> v_point_temp;
  v_point_temp.reserve(point_size);

  for (int i = 1; i < point_size; i++) {
    const auto &p = raw->points[i];
    const auto &p_prev = raw->points[i - 1];

    if ((p.line < num_scans) && (!is_invalid(p.x)) && (!is_invalid(p.y)) && (!is_invalid(p.z)) && p.x > 0.7) {
      if ((p.x > 2.0) && (((p.tag & 0x03) != 0x00) || ((p.tag & 0x0C) != 0x00))) continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(p.x, p.y, p.z);
      point_temp.point = point_temp.raw_point;
      // asuka time contract: offset_time is ns; offsets travel as milliseconds
      point_temp.relative_time = p.offset_time * 1e-6;
      point_temp.timestamp = point_temp.relative_time / 1000.0 + stamp;
      point_temp.alpha_time = 0.0;

      if ((std::abs(p.x - p_prev.x) > 1e-7) || (std::abs(p.y - p_prev.y) > 1e-7) ||
          (std::abs(p.z - p_prev.z) > 1e-7)) {
        v_point_temp.push_back(point_temp);
      }
    }
  }

  std::sort(v_point_temp.begin(), v_point_temp.end(), time_list);

  if (v_point_temp.empty()) return out;

  const double dt_last_point = v_point_temp.back().relative_time;

  // second pass: point_filter_num + blind sphere (r3live process)
  unsigned int num_valid = 0;
  for (size_t i = 0; i < v_point_temp.size(); i++) {
    num_valid++;
    if (num_valid % point_filter_num != 0) continue;

    const Eigen::Vector3d &raw_pt = v_point_temp[i].raw_point;
    if (raw_pt.x() * raw_pt.x() + raw_pt.y() * raw_pt.y() + raw_pt.z() * raw_pt.z() > (blind * blind)) {
      push_point(out, raw_pt, v_point_temp[i].relative_time, 0.0f);
    }
  }

  last_end_time = stamp + dt_last_point / 1000.0;
  return out;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr &raw) {
  (void)stamp;
  PointCloudT::Ptr out(new PointCloudT());
  const int size = static_cast<int>(raw->size());
  if (size == 0) return out;

  double dt_last_point = 0.0;

  const bool given_offset_time = raw->points[size - 1].timestamp > 0;
  if (!given_offset_time) {
    logger->warn("robosense scan without per-point timestamps is not supported");
    return out;
  }

  pcl::PointCloud<RobosensePoint> sorted(*raw);
  std::sort(sorted.points.begin(), sorted.points.end(),
            [](const RobosensePoint &a, const RobosensePoint &b) { return a.timestamp < b.timestamp; });
  dt_last_point = (sorted.points.back().timestamp - sorted.points.front().timestamp) * 1e-6;

  for (int i = 0; i < size; i++) {
    const auto &p = sorted.points[i];
    if (i % point_filter_num != 0) continue;

    const Eigen::Vector3d raw_pt(p.x, p.y, p.z);
    if (raw_pt.x() * raw_pt.x() + raw_pt.y() * raw_pt.y() + raw_pt.z() * raw_pt.z() > (blind * blind)) {
      const double offset_ms = (p.timestamp - sorted.points.front().timestamp) * 1e-6;
      push_point(out, raw_pt, offset_ms, p.intensity);
    }
  }

  // window times live in the header-stamp domain (upstream semantics:
  // timestamp = relative_time / 1000 + header stamp); emit only points past
  // the previous sweep end (upstream cross-scan gate).
  PointCloudT::Ptr gated(new PointCloudT());
  for (const auto &pt : out->points) {
    if (stamp + pt.curvature * 1e-3 > last_end_time) gated->points.push_back(pt);
  }
  last_end_time = stamp + dt_last_point / 1000.0;
  return gated;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr &raw) {
  assert(stamp > last_time_lidar);
  last_time_lidar = stamp;

  PointCloudT::Ptr out(new PointCloudT());
  const int point_size = static_cast<int>(raw->size());
  if (point_size < 2) return out;

  // first pass: r3live point selection (raw adjacent-pair dedup, no
  // point_filter_num/blind here -- they live in the second pass). Line-of-
  // sight hard blind (x > 0.7) and the tag mask mirror the CustomMsg path.
  std::vector<point3D> v_point_temp;
  v_point_temp.reserve(point_size);

  const double t0 = raw->empty() ? 0.0 : raw->points[0].timestamp;
  for (int i = 1; i < point_size; i++) {
    const auto &p = raw->points[i];
    const auto &p_prev = raw->points[i - 1];

    if ((p.line < num_scans) && (!is_invalid(p.x)) && (!is_invalid(p.y)) && (!is_invalid(p.z)) && p.x > 0.7) {
      if ((p.x > 2.0) && (((p.tag & 0x03) != 0x00) || ((p.tag & 0x0C) != 0x00))) continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(p.x, p.y, p.z);
      point_temp.point = point_temp.raw_point;
      // LivoxPoint carries absolute per-point timestamps (seconds): reduce
      // them to the scan time base so offsets travel as milliseconds, the
      // same contract as the CustomMsg path (driver offset_time ns -> ms).
      point_temp.relative_time = (p.timestamp - t0) * 1e3;
      point_temp.timestamp = point_temp.relative_time / 1000.0 + stamp;
      point_temp.alpha_time = 0.0;

      if ((std::abs(p.x - p_prev.x) > 1e-7) || (std::abs(p.y - p_prev.y) > 1e-7) ||
          (std::abs(p.z - p_prev.z) > 1e-7)) {
        v_point_temp.push_back(point_temp);
      }
    }
  }

  std::sort(v_point_temp.begin(), v_point_temp.end(), time_list);

  if (v_point_temp.empty()) return out;

  const double dt_last_point = v_point_temp.back().relative_time;

  // second pass: point_filter_num + blind sphere (r3live process)
  unsigned int num_valid = 0;
  for (size_t i = 0; i < v_point_temp.size(); i++) {
    num_valid++;
    if (num_valid % point_filter_num != 0) continue;

    const Eigen::Vector3d &raw_pt = v_point_temp[i].raw_point;
    if (raw_pt.x() * raw_pt.x() + raw_pt.y() * raw_pt.y() + raw_pt.z() * raw_pt.z() > (blind * blind)) {
      push_point(out, raw_pt, v_point_temp[i].relative_time, 0.0f);
    }
  }

  last_end_time = stamp + dt_last_point / 1000.0;
  return out;
}

}  // namespace srlivo
}  // namespace asuka
