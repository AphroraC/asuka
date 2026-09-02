#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/superlio/alias.hpp>
#include <asuka/odometry/superlio/cloud_preprocess.hpp>
#include <asuka/odometry/superlio/eskf.hpp>
#include <asuka/odometry/superlio/imu_preprocess.hpp>
#include <asuka/odometry/superlio/manifold.hpp>
#include <asuka/odometry/superlio/octvoxmap.hpp>
#include <asuka/odometry/superlio/timer.hpp>
#include <asuka/odometry/superlio/types.hpp>
#include <asuka/odometry/superlio/voxel_grid_closest.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace superlio {

class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override;

  OdometryEstimation(const OdometryEstimation&) = delete;
  OdometryEstimation& operator=(const OdometryEstimation&) = delete;

  void stop() override;
  void clear_buffers() override;
  PointCloudT::Ptr save_map() override;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) override;

  bool process_once() override;
  int workload() const override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override { return cloud_preprocess_impl; }

private:
  using StateFn = void (OdometryEstimation::*)();
  using OctVoxMapType = OctVoxMap<V3, Scalar>;
  using KnnHeapType = KnnHeap<5, V3>;

  bool sync_packages(MeasureGroup& measures);
  void state_wait_kf_init();
  void state_wait_map_init();
  void state_process();
  bool kf_init();
  bool map_init();
  void propagation_undistort();
  void down_sample();
  void observe();
  void update_map();
  void output();

  static inline bool calc_plane_coeff(int n, const std::array<V3, 5>& points, std::array<double, 4>& abcd);
  static inline bool compute_error(const std::array<double, 4>& abcd, const V3& point, float length,
                                   Scalar& error);

  std::shared_ptr<spdlog::logger> logger{nullptr};
  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  OctVoxMapType::Ptr ivox{nullptr};
  VoxelGridClosest<PointT> voxel_grid_filter;
  Eskf::Ptr kf{nullptr};

  StateFn state_fn{&OdometryEstimation::state_wait_kf_init};
  MeasureGroup measures;

  bool init_flag{false};
  bool first_scan_flag{true};
  std::vector<DynamicState, Eigen::aligned_allocator<DynamicState>> propagate_states;
  PointCloudT::Ptr scan_undistort_full{new PointCloudT()};
  PointCloudT::Ptr ds_undistort{new PointCloudT()};
  PointCloudT::Ptr world_pc{new PointCloudT()};
  PointCloudT::Ptr map_cloud{new PointCloudT()};

  Se3 sys_init_pose;
  Se3 last_pose;

  std::size_t effect_knn_num{0};
  VV3 points_world;
  VV3 points_body;
  alignas(64) bool effect_mask[20000] = {false};
  alignas(64) bool effect_knn_mask[20000] = {false};
  std::vector<int> effect_knn_idxs;
  std::vector<std::array<double, 4>> abcd_vec;

  Timer time_record;

  double gravity_norm{9.7946};
  double imu_na{0.1};
  double imu_ng{0.1};
  double imu_nba{0.0001};
  double imu_nbg{0.0001};
  int kf_max_iterations{4};
  bool kf_align_gravity{true};
  double kf_quit_eps{0.001};
  std::size_t ivox_capacity{2000000};
  float ivox_resolution{0.5f};
  float voxel_filter_size{0.5f};
  bool enable_downsample{true};
  bool time_eva{false};
  bool save_map_en{false};

  Se3 lidar_imu;
  Se3 odom_robo;
  M3 lidar_robo_yaw{M3::Identity()};

  mutable std::mutex buffer_mutex;
  boost::circular_buffer<LidarData> lidar_buffer;
  bool lidar_pushed{false};
  double last_timestamp_lidar{-1.0};

  int frame_num{0};
  long frame_counter{0};
};

}  // namespace superlio
}  // namespace asuka
