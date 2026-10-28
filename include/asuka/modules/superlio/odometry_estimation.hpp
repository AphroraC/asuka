#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/modules/superlio/alias.hpp>
#include <asuka/modules/superlio/cloud_preprocess.hpp>
#include <asuka/modules/superlio/eskf.hpp>
#include <asuka/modules/superlio/imu_preprocess.hpp>
#include <asuka/modules/superlio/manifold.hpp>
#include <asuka/modules/superlio/octvoxmap.hpp>
#include <asuka/modules/superlio/timer.hpp>
#include <asuka/modules/superlio/types.hpp>
#include <asuka/modules/superlio/voxel_grid_closest.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace superlio {

struct OdometryEstimationParam {
  bool save_map_en{false};

  // [kf] / [imu]
  int kf_max_iterations{4};
  bool kf_align_gravity{true};
  double kf_quit_eps{0.001};
  double gravity_norm{9.7946};
  double imu_na{0.1};
  double imu_ng{0.1};
  double imu_nba{0.0001};
  double imu_nbg{0.0001};

  // [hash_map] / [preprocess] / [eva] / [odometry]
  std::size_t ivox_capacity{2000000};
  float ivox_resolution{0.5f};
  float voxel_filter_size{0.5f};
  bool enable_downsample{true};
  bool time_eva{false};
  int lidar_buffer_capacity{50};
  int imu_buffer_capacity{1000};

  Se3 lidar_imu;
  Se3 odom_robo;
  Eigen::Matrix<float, 3, 3> lidar_robo_yaw{Eigen::Matrix<float, 3, 3>::Identity()};
};

class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override;

  OdometryEstimation(const OdometryEstimation&) = delete;
  OdometryEstimation& operator=(const OdometryEstimation&) = delete;

  void stop() override;
  PointCloudT::Ptr save_map() override;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) override;
  void insert_image(const ImageData::ConstPtr& image) override {}

  bool process_once() override;
  int workload() const override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override { return cloud_preprocess_impl; }

private:
  using StateFn = void (OdometryEstimation::*)();
  using OctVoxMapType = OctVoxMap<Eigen::Matrix<float, 3, 1>, float>;
  using KnnHeapType = KnnHeap<5, Eigen::Matrix<float, 3, 1>>;

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

  static inline bool
  calc_plane_coeff(int n, const std::array<Eigen::Matrix<float, 3, 1>, 5>& points, std::array<double, 4>& abcd);
  static inline bool
  compute_error(const std::array<double, 4>& abcd, const Eigen::Matrix<float, 3, 1>& point, float length, float& error);

  std::shared_ptr<spdlog::logger> logger{nullptr};
  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  OdometryEstimationParam param;

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
  std::vector<Eigen::Matrix<float, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 1>>> points_world;
  std::vector<Eigen::Matrix<float, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 1>>> points_body;
  alignas(64) bool effect_mask[20000] = {false};
  alignas(64) bool effect_knn_mask[20000] = {false};
  std::vector<int> effect_knn_idxs;
  std::vector<std::array<double, 4>> abcd_vec;

  Timer time_record;

  mutable std::mutex buffer_mutex;
  boost::circular_buffer<LidarData> lidar_buffer;
  bool lidar_pushed{false};
  double last_timestamp_lidar{-1.0};

  int frame_num{0};
  long frame_counter{0};
};

}  // namespace superlio
}  // namespace asuka
