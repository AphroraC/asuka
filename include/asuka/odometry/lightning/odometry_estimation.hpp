#pragma once

#include <atomic>
#include <cmath>
#include <list>
#include <memory>
#include <mutex>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <common/eigen_types.h>
#include <common/keyframe.h>
#include <common/measure_group.h>
#include <common/nav_state.h>
#include <common/options.h>
#include <common/point_def.h>
#include <core/ivox3d/ivox3d.h>
#include <core/lio/eskf.hpp>
#include <core/lio/imu_processing.hpp>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/lightning/cloud_preprocess.hpp>
#include <asuka/odometry/lightning/imu_preprocess.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace lightning {

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
  bool sync_packages(ImuMeasureGroup& measures);
  void process_scan(ImuMeasureGroup& measures);
  void obs_model(::lightning::NavState& s, ::lightning::ESKF::CustomObservationModel& obs);
  void point_body_to_world(const ::lightning::PointType& pi, ::lightning::PointType& po);
  void map_incremental();
  void make_kf();
  ::lightning::CloudPtr get_global_map(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

private:
  using IVoxType = ::lightning::IVox<3, ::lightning::IVoxNodeType::DEFAULT, ::lightning::PointType>;

  std::shared_ptr<spdlog::logger> logger{nullptr};
  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  IVoxType::Options ivox_options;
  std::shared_ptr<IVoxType> ivox{nullptr};

  ::lightning::ESKF kf;
  ::lightning::ESKF kf_imu;
  ::lightning::NavState state_point;

  ::lightning::Mat3d offset_r_lidar_fixed{::lightning::Mat3d::Identity()};
  ::lightning::Vec3d offset_t_lidar_fixed{::lightning::Vec3d::Zero()};

  double filter_size_map_min{0.0};
  pcl::VoxelGrid<::lightning::PointType> voxel_scan;

  std::vector<::lightning::Keyframe::Ptr> all_keyframes;
  ::lightning::Keyframe::Ptr last_kf{nullptr};
  std::list<::lightning::Keyframe::Ptr> proj_kfs;
  int kf_id{0};

  ::lightning::CloudPtr scan_undistort{new ::lightning::PointCloudType()};
  ::lightning::CloudPtr scan_down_body{new ::lightning::PointCloudType()};
  ::lightning::CloudPtr scan_down_world{new ::lightning::PointCloudType()};

  std::vector<::lightning::PointVector> nearest_points;
  std::vector<::lightning::Vec4f> corr_pts;
  std::vector<::lightning::Vec4f> corr_norm;
  std::vector<float> residuals;
  std::vector<char> point_selected_surf;
  std::vector<char> point_selected_icp;
  std::vector<::lightning::Vec4f> plane_coef;

  std::mutex mtx_buffer;
  boost::circular_buffer<PointCloudT::Ptr> lidar_buffer;
  boost::circular_buffer<double> time_buffer;
  boost::circular_buffer<::lightning::IMUPtr> imu_buffer;

  bool lidar_pushed{false};
  double lidar_end_time{0.0};
  double last_timestamp_imu{-1.0};
  double last_timestamp_lidar{0.0};
  double first_lidar_time{0.0};
  double last_lidar_time{0.0};
  double lidar_mean_scantime{0.0};
  int scan_num{0};

  bool enable_skip_lidar{true};
  int skip_lidar_num{5};
  int skip_lidar_cnt{0};

  bool first_scan_flag{true};
  bool ekf_inited_flag{false};
  int scan_count{0};
  int effect_feat_surf{0};
  int effect_feat_icp{0};

  ::lightning::MeasureGroup measures;

  bool use_aa{false};
  bool keep_first_imu_estimation{false};

  double kf_dis_th{2.0};
  double kf_angle_th{15.0 * M_PI / 180.0};
  bool enable_icp_part{true};
  double plane_icp_weight{1.0};
  double icp_weight{100.0};
  int min_pts{300};
  bool proj_kfs_en{false};
  int max_proj_kfs{5};

  long frame_counter{0};

  std::atomic<bool> kill_switch{false};
};

}  // namespace lightning
}  // namespace asuka
