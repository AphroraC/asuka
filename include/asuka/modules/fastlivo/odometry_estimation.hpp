#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>

#include <vikit/abstract_camera.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/modules/fastlivo/cloud_preprocess.hpp>
#include <asuka/modules/fastlivo/common_lib.hpp>
#include <asuka/modules/fastlivo/imu_preprocess.hpp>
#include <asuka/modules/fastlivo/vio.hpp>
#include <asuka/modules/fastlivo/voxel_map.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlivo {

struct OdometryEstimationParam {
  SLAM_MODE slam_mode{ONLY_LIO};

  // imu
  double gyr_cov{1.0};
  double acc_cov{1.0};
  double inv_expo_cov{0.2};
  int imu_int_frame{3};
  bool gravity_est_en{true};
  bool ba_bg_est_en{true};
  bool exposure_estimate_en{true};

  // lio
  double filter_size_surf{0.5};

  // vio
  bool normal_en{true};
  bool inverse_composition_en{false};
  bool raycast_en{false};
  int max_iterations{5};
  double img_point_cov{100.0};
  int grid_size{5};
  int grid_n_height{17};
  int patch_size{8};
  int patch_pyrimid_level{3};
  double outlier_threshold{1000.0};

  // time_offset
  double img_time_offset{0.0};
  double exposure_time_init{0.0};

  // publish
  double blind_rgb_points{0.01};
  // World cloud built from the full dense scan instead of the downsampled
  // one (drives the save accumulation and the VIO coloring input).
  bool dense_map_en{false};

  // preprocess
  bool hilti_en{false};

  // map saving (config_ros enable_map_saving, framework-wide switch)
  bool save_map_en{false};

  // bounded input queues (upstream: unbounded deques)
  int imu_buffer_capacity{1000};
  int lidar_buffer_capacity{50};
  int image_buffer_capacity{50};
  int measures_capacity{50};
};

class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override;

  OdometryEstimation(const OdometryEstimation&) = delete;
  OdometryEstimation& operator=(const OdometryEstimation&) = delete;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) override;
  void insert_image(const ImageData::ConstPtr& image) override;

  int workload() const override;
  void stop() override;
  PointCloudT::Ptr save_map() override;
  bool process_once() override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override { return cloud_preprocess_impl; }

private:
  bool sync_packages(LidarMeasureGroup& meas);
  void handle_first_frame();
  void process_imu();
  void state_estimation_and_mapping();
  void handle_lio();
  void handle_vio();

  void emit_key_frame(double stamp, const PointCloudT::Ptr& cloud_imu);
  void colorize_cloud(PointCloudT& cloud);
  void accumulate_colored_map();
  void transform_lidar(
    const Eigen::Matrix3d rot,
    const Eigen::Vector3d t,
    const PointCloudT::Ptr& input_cloud,
    PointCloudT::Ptr& trans_cloud);
  void point_body_to_world(const PointT& pi, PointT& po);
  vk::AbstractCamera* make_camera(const Config& config);

  OdometryEstimationParam param;

  mutable std::mutex buffer_mutex;

  boost::circular_buffer<PointCloudT::Ptr> lidar_buffer;
  boost::circular_buffer<double> lidar_time_buffer;
  boost::circular_buffer<ImuData::ConstPtr> imu_buffer;
  boost::circular_buffer<ImageData::ConstPtr> img_buffer;
  boost::circular_buffer<double> img_time_buffer;

  double last_timestamp_lidar{-1.0};
  double last_timestamp_imu{-1.0};
  double last_timestamp_img{-1.0};
  bool lidar_pushed{false};
  int hilti_frame_counter{0};
  uint64_t livo_wait_count{0};

  double first_lidar_time{0.0};
  bool is_first_frame{false};
  bool lidar_map_inited{false};
  long frame_counter{0};

  Eigen::Vector3d extT{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d extR{Eigen::Matrix3d::Identity()};
  std::vector<double> cameraextrinT;
  std::vector<double> cameraextrinR;
  std::shared_ptr<vk::AbstractCamera> camera;

  LidarMeasureGroup lidar_measures;
  StatesGroup state;
  StatesGroup state_propagat;
  std::vector<pointWithVar> pv_list;
  int feats_down_size{0};

  PointCloudT::Ptr feats_undistort{new PointCloudT()};
  PointCloudT::Ptr feats_down_body{new PointCloudT()};
  PointCloudT::Ptr feats_down_world{new PointCloudT()};
  PointCloudT::Ptr pcl_w_wait_pub{new PointCloudT()};
  PointCloudT::Ptr pcl_wait_pub{new PointCloudT()};
  PointCloudT::Ptr pcl_wait_save{new PointCloudT()};
  PointCloudT::Ptr current_cloud_imu{nullptr};

  pcl::VoxelGrid<PointT> down_size_filter_surf;
  Eigen::Vector3d euler_cur{Eigen::Vector3d::Zero()};

  std::shared_ptr<ImuPreprocess> p_imu{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> voxel_map;
  std::shared_ptr<VoxelMapManager> voxelmap_manager{nullptr};
  std::shared_ptr<VIOManager> vio_manager{nullptr};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace fastlivo
}  // namespace asuka
