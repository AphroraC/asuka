#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <Scancontext.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace optimization_loam {

using PointType = asuka::PointT;

struct Pose6D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

gtsam::Pose3 pose6d_to_gtsam(const Pose6D& p);
Pose6D diff_transformation(const Pose6D& p1, const Pose6D& p2);
pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloud_in, const Pose6D& tf);
Eigen::Affine3f pose6d_to_affine(const Pose6D& pose);

// SC-A-LOAM style backend: subscribes frontend keyframes, runs ScanContext
// + distance loop detection, ICP verification and incremental gtsam ISAM2
// optimization, and emits the optimized trajectory.
class OptimizationLOAM : public ExtensionModule {
public:
  OptimizationLOAM();
  ~OptimizationLOAM() override;
  void stop() override;

private:
  void on_new_frame(const KeyFrame::ConstPtr& frame);
  void input_thread();
  void publish_loop();

  void add_keyframe(double stamp, const Pose6D& pose, const pcl::PointCloud<PointType>::Ptr& cloud);
  void init_noises();
  void run_isam2opt();
  void update_poses();
  void loop_find_near_keyframes(pcl::PointCloud<PointType>::Ptr& near_keyframes, int key, int search_num);
  gtsam::Pose3 do_icp_virtual_relative(int loop_kf_idx, int curr_kf_idx);
  void perform_sc_loop_closure();
  void perform_rs_loop_closure();
  bool detect_loop_closure_distance(int* loop_key_cur, int* loop_key_pre);
  void process_lcd();
  void process_icp();
  void process_isam();
  void process_viz_map();
  bool get_latest_pose(double& stamp, Pose6D& pose) const;

  struct InputFrame {
    double stamp{0.0};
    Pose6D pose{};
    PointCloudT::Ptr cloud{nullptr};
  };

  int on_new_frame_id{-1};

  static constexpr std::size_t queue_capacity = 50;

  // The voxel downsample and ScanContext descriptor build run on the input
  // thread; the on_new_frame callback only enqueues, so the odometry worker
  // is never blocked by the backend.
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<InputFrame> queue{queue_capacity};
  std::size_t dropped_items{0};

  mutable std::mutex m_buf;
  mutable std::mutex m_kf;
  mutable std::mutex mtx_posegraph;

  std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_laser_clouds;
  std::vector<Pose6D> keyframe_poses;
  std::vector<Pose6D> keyframe_poses_updated;
  std::vector<double> keyframe_times;
  int recent_idx_updated{0};

  std::map<int, int> loop_index_container;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_key_poses{new pcl::KdTreeFLANN<pcl::PointXYZ>()};

  gtsam::NonlinearFactorGraph gt_sam_graph;
  bool gt_sam_graph_made{false};
  gtsam::Values initial_estimate;
  std::unique_ptr<gtsam::ISAM2> isam;
  gtsam::Values isam_current_estimate;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
  gtsam::noiseModel::Base::shared_ptr robust_loop_noise;

  pcl::VoxelGrid<PointType> downsize_filter_scancontext;
  SCManager sc_manager;

  pcl::VoxelGrid<PointType> downsize_filter_icp;
  pcl::PointCloud<PointType>::Ptr laser_cloud_map_pgo{new pcl::PointCloud<PointType>()};
  pcl::VoxelGrid<PointType> downsize_filter_map_pgo;
  std::atomic<std::size_t> map_seq{0};

  double translation_accumulated{1000000.0};
  double rotation_accumulated{1000000.0};
  Pose6D odom_pose_prev{};
  Pose6D odom_pose_curr{};

  std::queue<std::pair<int, int>> sc_loop_icp_buf;

  double keyframe_meter_gap{2.0};
  double keyframe_deg_gap{10.0};
  double keyframe_rad_gap{0.1745};
  double sc_dist_thres{0.2};
  double sc_max_radius{80.0};
  double history_keyframe_search_radius{10.0};
  double history_keyframe_search_time_diff{30.0};
  int history_keyframe_search_num{25};
  double loop_noise_score{0.5};
  int graph_update_times{2};
  double loop_fitness_score_threshold{0.3};
  double loop_closure_frequency{2.0};
  double graph_update_frequency{1.0};
  double vizmap_frequency{0.1};

  double publish_interval{0.1};
  std::atomic<bool> kill_switch{false};
  std::thread input_thread_obj;
  std::thread publish_thread;
  std::thread lcd_thread;
  std::thread icp_thread;
  std::thread isam_thread;
  std::thread viz_map_thread;

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace optimization_loam
}  // namespace asuka
