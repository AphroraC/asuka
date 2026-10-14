#include <asuka/odometry/smallpointlio/cloud_preprocess.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <asuka/utility/logging.hpp>

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
}

PointCloudT::Ptr CloudPreprocess::build(double stamp, const pcl::PointCloud<LivoxPoint>& raw) const {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw.empty()) return output;

  const double timebase = raw.points[0].timestamp;
  output->reserve(raw.size());
  for (const auto& pt : raw.points) {
    // Same tag filter as the upstream livox adapters; points are kept in
    // message order because the downstream filtering indexes this sequence.
    if ((pt.tag & 0b00111111) != 0b00000000) continue;
    PointT point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = 0.0f;
    point.curvature = static_cast<float>((pt.timestamp - timebase) * 1e-6);
    output->push_back(point);
  }
  output->width = output->size();

  logger->debug("Livox converted {} points from {} input points", output->size(), raw.size());
  return output;
}

PointCloudT::Ptr CloudPreprocess::build(double stamp, const pcl::PointCloud<RobosensePoint>& raw) const {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw.empty()) return output;

  const double timebase = raw.points[0].timestamp;
  output->reserve(raw.size());
  for (const auto& pt : raw.points) {
    PointT point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = 0.0f;
    point.curvature = static_cast<float>((pt.timestamp - timebase) * 1e-6);
    output->push_back(point);
  }
  output->width = output->size();

  logger->debug("Robosense converted {} points from {} input points", output->size(), raw.size());
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

PointCloudT::Ptr CloudPreprocess::preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) {
  (void)stamp;
  PointCloudT::Ptr output(new PointCloudT);
  if (raw->empty()) return output;
  output->reserve(raw->size());
  for (const auto& pt : raw->points) {
    if ((pt.tag & 0b00111111) != 0b00000000) continue;
    PointT point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    point.intensity = 0.0f;
    point.curvature = static_cast<float>(pt.offset_time * 1e-6);
    output->push_back(point);
  }
  output->width = output->size();

  logger->debug("Livox CustomMsg converted {} points from {} input points", output->size(), raw->size());
  return output;
}

}  // namespace smallpointlio

}  // namespace asuka
