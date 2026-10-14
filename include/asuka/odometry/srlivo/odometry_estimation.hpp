#pragma once
// asuka::srlivo -- port of sr_livo (COIN-LIO-style CT-ICP odometry with the
// R3LIVE visual rendering branch) as an asuka odometry plugin. Upstream is a
// single livo_node class driven by a 200 Hz polling loop; here the three
// sensor streams arrive through the asuka executor and process_once()
// advances the same measurement-window state machine.

#include <memory>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/srlivo/cloud_map.hpp>
#include <asuka/odometry/srlivo/cloud_preprocess.hpp>
#include <asuka/odometry/srlivo/eskf_estimator.hpp>
#include <asuka/odometry/srlivo/frame.hpp>
#include <asuka/odometry/srlivo/image_processing.hpp>
#include <asuka/odometry/srlivo/state.hpp>
#include <asuka/odometry/srlivo/utility.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace srlivo {

// Ported from sr_livo include/parameters.h (recordParameters debugging and
// the unused option presets are not migrated).
class icpOptions {
public:
  int threshold_voxel_occupancy = 1;
  int init_num_frames = 20;             // frames defining the map initialization
  double size_voxel_map = 1.0;          // max voxel: +-32 km for 1 m voxels
  int num_iters_icp = 5;                // maximum ICP iterations
  int min_number_neighbors = 20;
  int voxel_neighborhood = 1;           // visits the (3 * voxel_neighborhood)^3 neighboring voxels
  double power_planarity = 2.0;         // power of the planarity weighting
  bool estimate_normal_from_neighborhood = true;
  int max_number_neighbors = 20;
  double max_dist_to_plane_icp = 0.3;
  double threshold_orientation_norm = 0.0001;  // deg, ICP early termination
  double threshold_translation_norm = 0.001;   // m, ICP early termination
  bool point_to_plane_with_distortion = true;  // distort the frame at each ICP iteration
  int max_num_residuals = -1;
  int min_num_residuals = 100;
  int num_closest_neighbors = 1;
  double weight_alpha = 0.9;
  double weight_neighborhood = 0.1;
  bool debug_print = true;
};

class odometryOptions {
public:
  double init_voxel_size = 0.2;
  double init_sample_voxel_size = 1.0;
  int init_num_frames = 20;
  int num_for_initialization = 10;
  double voxel_size = 0.5;
  double sample_voxel_size = 1.5;
  double max_distance = 100.0;
  int max_num_points_in_voxel = 20;
  double min_distance_points = 0.1;
  double distance_error_threshold = 5.0;

  icpOptions optimize_options;

  MotionCompensation motion_compensation = CONSTANT_VELOCITY;
  StateInitialization initialization = INIT_CONSTANT_VELOCITY;
};

class mapOptions {
public:
  double size_voxel_map = 0.1;
  int max_num_points_in_voxel = 20;
  double min_distance_points = 0.01;
  int add_point_step = 4;
  int pub_point_minimum_views = 3;
};

struct OdometryEstimationParam {
  // bounded rings (asuka adaptation of the upstream unbounded queues)
  int point_buffer_capacity{100000};
  int imu_buffer_capacity{5000};
  int image_buffer_capacity{50};

  // preprocess
  int scan_line{6};   // upstream N_SCANS
  int scan_rate{10};  // upstream SCAN_RATE: sweep interval = 1 / scan_rate
  double blind{0.1};
  int point_filter_num{1};

  // imu
  double acc_cov{0.1};
  double gyr_cov{0.1};
  double b_acc_cov{0.0001};
  double b_gyr_cov{0.0001};
  Eigen::Vector3d gravity_acc{0.0, 0.0, 9.81};

  // camera
  int image_width{1280};
  int image_height{1024};
  Eigen::Matrix3d camera_intrinsic{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 5, 1> camera_dist_coeffs{Eigen::Matrix<double, 5, 1>::Zero()};
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_imu_camera{Eigen::Vector3d::Zero()};

  // lidar extrinsic
  Eigen::Matrix3d R_imu_lidar{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_imu_lidar{Eigen::Vector3d::Zero()};

  // map saving gate (config_ros, shared asuka contract)
  bool save_map_en{false};

  void load(const std::shared_ptr<spdlog::logger> &logger, odometryOptions &odometry_options, mapOptions &map_options);
};

class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override;

  void insert_imu(const ImuData::ConstPtr &imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr &cloud) override;
  // The visual branch is the algorithm itself: images drive the measurement
  // windows. Without an image stream no measurement is ever produced.
  void insert_image(const ImageData::ConstPtr &image) override;

  int workload() const override;
  void stop() override;
  PointCloudT::Ptr save_map() override;
  bool process_once() override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override;

private:
  // main loop
  std::vector<Measurements> getMeasurements();
  void runMeasurements(std::vector<Measurements> &measurements);
  void process(std::vector<point3D> &cut_sweep, double timestamp_begin, double timestamp_offset, const ImageData::ConstPtr &cur_image, bool to_rendering);

  // data handle and state estimation
  optimizeSummary optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, double sample_voxel_size);
  optimizeSummary buildPlaneResiduals(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints,
                                      std::vector<planeParam> &plane_residuals, cloudFrame *p_frame, double &loss_sum);
  optimizeSummary updateIEKF(const icpOptions &cur_icp_options, voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame);
  cloudFrame *buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset);
  void makePointTimestamp(std::vector<point3D> &sweep, double time_begin, double time_end);
  void stateInitialization(state *cur_state);
  optimizeSummary stateEstimation(cloudFrame *p_frame, bool to_rendering);
  Neighborhood computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(voxelHashMap &map, const Eigen::Vector3d &point,
                                                                                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

  // map update
  void addPointToMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points,
                     int min_num_points, cloudFrame *p_frame);
  void addPointToColorMap(voxelHashMap &map, rgbPoint &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points,
                          int min_num_points, cloudFrame *p_frame, std::vector<voxelId> &voxels_recent_visited_temp);
  void addPointsToMap(voxelHashMap &map, cloudFrame *p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0, bool to_rendering = false);

  // asuka surface: KeyFrame emission (replaces the upstream publish block)
  void emitKeyFrame(cloudFrame *p_frame, bool rendering);

  OdometryEstimationParam param;
  std::shared_ptr<spdlog::logger> logger;

  std::shared_ptr<CloudPreprocess> cloud_preprocess_impl;
  std::unique_ptr<eskfEstimator> eskf_pro;
  std::unique_ptr<imageProcessing> img_pro;

  double laser_point_cov{0.001};

  Eigen::Vector3d G{Eigen::Vector3d::Zero()};
  double G_norm{9.81};

  Eigen::Matrix3d R_imu_lidar{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_imu_lidar{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_imu_camera{Eigen::Vector3d::Zero()};

  boost::circular_buffer<point3D> point_buffer{1};
  boost::circular_buffer<ImuData::ConstPtr> imu_buffer{1};
  boost::circular_buffer<ImageData::ConstPtr> img_buffer{1};
  boost::circular_buffer<double> img_time_buffer{1};

  std::vector<cloudFrame *> all_cloud_frame;

  odometryOptions odometry_options;
  mapOptions map_options;

  bool initial_flag{false};
  bool kill_switch{false};
  bool point_time_enable{true};  // both asuka handlers provide per-point offsets

  double last_time_lidar{-1.0};
  double last_time_imu{-1.0};
  double last_time_img{-1.0};
  double last_get_measurement{-1.0};
  double last_time_frame{-1.0};
  double current_time{-1.0};
  double dt_sum{0.0};

  int index_frame{1};
  long keyframe_counter{0};
  int missing_image_warn_count{0};

  // running points-per-frame estimate, for the frames-equivalent workload()
  double points_per_frame{20000.0};

  std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_meas;
  std::vector<imuState> imu_states;

  std::vector<voxelId> voxels_recent_visited_temp;

  voxelHashMap voxel_map;
  voxelHashMap color_voxel_map;
  Hash_map_3d<long, rgbPoint *> hashmap_3d_points;
};

}  // namespace srlivo
}  // namespace asuka
