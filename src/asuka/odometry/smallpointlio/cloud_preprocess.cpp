#include <asuka/odometry/smallpointlio/cloud_preprocess.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <asuka/utility/logging.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace smallpointlio {
namespace {

inline Eigen::Array3i fast_floor(const Eigen::Array3f& pt) {
  const Eigen::Array3i ncoord = pt.cast<int>();
  return ncoord - (pt < ncoord.cast<float>()).cast<int>();
}

}  // namespace

CloudPreprocess::CloudPreprocess() {
  logger = create_module_logger("preprocess");
  const Config config(GlobalConfig::get_config_path("config_odometry"));
  min_distance = config.param<double>("preprocess", "min_distance", 0.5);
  max_distance = config.param<double>("preprocess", "max_distance", 100.0);
  point_filter_num = std::max(1, config.param<int>("preprocess", "point_filter_num", 1));
  space_downsample = config.param<bool>("preprocess", "space_downsample", false);
  space_downsample_leaf_size = config.param<double>("preprocess", "space_downsample_leaf_size", 0.5);
}

PointCloudT::Ptr CloudPreprocess::build(double stamp, const pcl::PointCloud<LivoxPoint>& raw) const {
  PointCloudT::Ptr output(new PointCloudT);
  if (raw.empty()) return output;

  std::vector<RawPoint> points;
  points.reserve(raw.size());
  const double timebase = raw.points[0].timestamp;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto& pt = raw.points[i];
    if ((pt.tag & 0b00111111) != 0b00000000 || i % point_filter_num != 0) continue;
    const Eigen::Vector3f position(pt.x, pt.y, pt.z);
    const double distance = position.squaredNorm();
    if (!position.allFinite() || distance < min_distance * min_distance || distance > max_distance * max_distance)
      continue;
    points.push_back(RawPoint{(pt.timestamp - timebase) * 1e-6, position});
  }
  std::vector<RawPoint> sampled;
  if (space_downsample) {
    VoxelgridSampling sampler;
    sampler.voxelgrid_sampling(points, sampled, space_downsample_leaf_size);
  } else {
    sampled = std::move(points);
  }
  std::sort(sampled.begin(), sampled.end(), [](const RawPoint& lhs, const RawPoint& rhs) {
    return lhs.timestamp < rhs.timestamp;
  });
  output->reserve(sampled.size());
  for (const auto& point : sampled) {
    PointT p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    p.intensity = 0.0f;
    p.curvature = static_cast<float>(point.timestamp);
    output->push_back(p);
  }

  logger->debug("Livox preprocessed {} points from {} input points", output->size(), raw.size());
  return output;
}

PointCloudT::Ptr CloudPreprocess::build(double stamp, const pcl::PointCloud<RobosensePoint>& raw) const {
  PointCloudT::Ptr output(new PointCloudT);
  if (raw.empty()) return output;

  std::vector<RawPoint> points;
  points.reserve(raw.size());
  const double timebase = raw.points[0].timestamp;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto& pt = raw.points[i];
    if (i % point_filter_num != 0) continue;
    const Eigen::Vector3f position(pt.x, pt.y, pt.z);
    const double distance = position.squaredNorm();
    if (!position.allFinite() || distance < min_distance * min_distance || distance > max_distance * max_distance)
      continue;
    points.push_back(RawPoint{(pt.timestamp - timebase) * 1e-6, position});
  }
  std::vector<RawPoint> sampled;
  if (space_downsample) {
    VoxelgridSampling sampler;
    sampler.voxelgrid_sampling(points, sampled, space_downsample_leaf_size);
  } else {
    sampled = std::move(points);
  }
  std::sort(sampled.begin(), sampled.end(), [](const RawPoint& lhs, const RawPoint& rhs) {
    return lhs.timestamp < rhs.timestamp;
  });
  output->reserve(sampled.size());
  for (const auto& point : sampled) {
    PointT p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    p.intensity = 0.0f;
    p.curvature = static_cast<float>(point.timestamp);
    output->push_back(p);
  }

  logger->debug("Robosense preprocessed {} points from {} input points", output->size(), raw.size());
  return output;
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  return build(stamp, *raw);
}

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  return build(stamp, *raw);
}

void VoxelgridSampling::voxelgrid_sampling(
  const std::vector<RawPoint>& points,
  std::vector<RawPoint>& downsampled,
  double leaf_size) {
  if (points.empty()) {
    downsampled = points;
    return;
  }

  const double inv_leaf_size = 1.0 / leaf_size;
  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size = 21;
  constexpr size_t coord_bit_mask = (1 << 21) - 1;
  constexpr int coord_offset = 1 << (coord_bit_size - 1);

  coord_pt.resize(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Array3i coord = fast_floor(points[i].position * inv_leaf_size) + coord_offset;
    if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
      coord_pt[i] = {invalid_coord, i};
      continue;
    }
    const std::uint64_t bits = (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |
                               (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |
                               (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
    coord_pt[i] = {bits, i};
  }

  const auto compare = [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; };
  std::sort(coord_pt.begin(), coord_pt.end(), compare);

  downsampled.clear();
  size_t i = 0;
  while (i < coord_pt.size()) {
    if (coord_pt[i].first == invalid_coord) {
      ++i;
      continue;
    }
    const std::uint64_t voxel = coord_pt[i].first;
    RawPoint sum = points[coord_pt[i].second];
    size_t count = 1;
    ++i;
    while (i < coord_pt.size() && coord_pt[i].first == voxel) {
      const auto& point = points[coord_pt[i].second];
      sum.position += point.position;
      sum.timestamp = point.timestamp;
      ++count;
      ++i;
    }
    sum.position /= static_cast<float>(count);
    downsampled.push_back(sum);
  }
}

}  // namespace smallpointlio
}  // namespace asuka
