#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/batchlio/cloud_preprocess.hpp>
#include <asuka/odometry/batchlio/imu_preprocess.hpp>
#include <asuka/odometry/batchlio/common.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace batchlio {

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
  void reset();
  bool synchronize(batch::MeasureGroup& measurements);
  void process(const batch::MeasureGroup& measurements);

  void handle_first_scan(const batch::MeasureGroup& measurements);
  void downsample_and_window(const batch::MeasureGroup& measurements);
  void initialize_map(const batch::MeasureGroup& measurements);
  void run_output_loop(const batch::MeasureGroup& measurements);
  void run_input_loop(const batch::MeasureGroup& measurements);
  void map_incremental();
  void emit_outputs(const batch::MeasureGroup& measurements);

  void point_body_to_world(const PointT& pi, PointT& po);
  Eigen::Matrix<double, 24, 24> process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> process_noise_cov_output();
  void h_model_input(batch::state_input& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_r,
                     esekfom::dyn_share_modified<double>& ekfom_data);
  void h_model_output(batch::state_output& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_r,
                      esekfom::dyn_share_modified<double>& ekfom_data);
  void h_model_imu_output(batch::state_output& s, esekfom::dyn_share_modified<double>& ekfom_data);

  static void h_model_input_trampoline(batch::state_input& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_r,
                                       esekfom::dyn_share_modified<double>& ekfom_data);
  static void h_model_output_trampoline(batch::state_output& s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_r,
                                         esekfom::dyn_share_modified<double>& ekfom_data);
  static void h_model_imu_output_trampoline(batch::state_output& s,
                                             esekfom::dyn_share_modified<double>& ekfom_data);

  static thread_local OdometryEstimation* active;

  std::mutex buffer_mutex;
  boost::circular_buffer<PointCloudT::ConstPtr> lidar_buffer;
  boost::circular_buffer<double> time_buffer;
  boost::circular_buffer<ImuData::ConstPtr> imu_buffer;
  std::vector<ImuData::ConstPtr> imu_initialization;
  ImuData::ConstPtr last_synchronized_imu{nullptr};
  double last_imu_stamp{-1.0};
  double last_lidar_stamp{-1.0};
  bool initializing_imu{true};

  boost::circular_buffer<ImuData::ConstPtr> imu_deque;
  ImuData imu_last;
  ImuData imu_next;
  double last_queued_imu_stamp{-1.0};

  esekfom::esekf<batch::state_input, 24, batch::input_ikfom> kf_input;
  esekfom::esekf<batch::state_output, 30, batch::input_ikfom> kf_output;
  batch::IVoxType::Options ivox_options;
  std::shared_ptr<batch::IVoxType> ivox{nullptr};

  PointCloudT::Ptr normvec{new PointCloudT(100000, 1)};
  std::vector<int> time_seq;
  PointCloudT::Ptr feats_down_body{new PointCloudT(10000, 1)};
  PointCloudT::Ptr feats_down_world{new PointCloudT(10000, 1)};
  std::vector<Eigen::Vector3d> pbody_list;
  std::vector<PointVectorT> nearest_points;
  std::vector<float> point_search_sq_dis;
  std::array<bool, 100000> point_selected_surf{};
  std::vector<Eigen::Matrix3d> crossmat_list;
  int effct_feat_num{0};
  int k_window{0};
  int idx{-1};
  Eigen::Vector3d angvel_avr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_avr{Eigen::Vector3d::Zero()};
  int feats_down_size{0};
  batch::input_ikfom input_in;

  PointCloudT::Ptr feats_undistort{new PointCloudT};
  PointCloudT::Ptr init_feats_world{new PointCloudT};
  bool init_map{false};
  bool first_scan_flag{true};
  bool is_first_frame{true};
  bool reset_flag{false};
  double first_lidar_time{0.0};
  double first_imu_time{0.0};
  double time_current{0.0};
  double time_update_last{0.0};
  double time_predict_last_const{0.0};
  double t_last{0.0};

  Eigen::Matrix<double, 24, 24> p_input;
  Eigen::Matrix<double, 30, 30> p_output;
  Eigen::Matrix<double, 24, 24> q_input;
  Eigen::Matrix<double, 30, 30> q_output;

  Eigen::Vector3d gravity_vec{0.0, 0.0, -9.80665};
  Eigen::Vector3d gravity_init_vec{0.0, 0.0, -9.80665};

  bool pcd_save_en{false};
  PointCloudT::Ptr map_cloud{new PointCloudT};

  pcl::VoxelGrid<PointT> downsize_filter_surf;
  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  Eigen::Vector3d extrinsic_T{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d extrinsic_R{Eigen::Matrix3d::Identity()};

  bool imu_enabled{true};
  bool prop_at_freq_of_imu{true};
  bool check_satu{true};
  int omp_threads{1};
  double satu_acc{3.0};
  double satu_gyro{35.0};
  double acc_norm{1.0};
  float plane_thr{0.1f};
  double match_s{81.0};
  double laser_point_cov{0.01};
  double imu_meas_omg_cov{0.1};
  double imu_meas_acc_cov{0.1};
  double gyr_cov_input{0.01};
  double acc_cov_input{0.1};
  double vel_cov{20.0};
  double gyr_cov_output{1000.0};
  double acc_cov_output{500.0};
  double b_gyr_cov{0.0001};
  double b_acc_cov{0.0001};
  double batch_dt{0.001};
  bool batch_deskew{true};
  bool batch_omp{false};
  double lidar_time_inte{0.1};
  bool space_down_sample{true};
  double filter_size_surf_min{0.5};
  double filter_size_map_min{0.5};
  int init_map_size{100};
  bool use_imu_as_input{false};
  double g_m_s2{9.81};

  long frame_counter{0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace batchlio
}  // namespace asuka
