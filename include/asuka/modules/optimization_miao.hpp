#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <common/eigen_types.h>
#include <common/std_types.h>
#include <core/graph/optimizer.h>
#include <core/opti_algo/algo_select.h>
#include <core/robust_kernel/cauchy.h>
#include <core/types/edge_se3.h>
#include <core/types/edge_se3_height_prior.h>
#include <core/types/vertex_se3.h>

#include <sophus/se3.hpp>

#include <spdlog/spdlog.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace optimization_miao {

using PointType = asuka::PointT;
using SE3d = ::lightning::SE3;
using Mat6d = ::lightning::Mat6d;
using Mat3d = ::lightning::Mat3d;
using Vec3d = ::lightning::Vec3d;
using Vec4d = ::lightning::Vec4d;

struct LoopCandidate {
  LoopCandidate() = default;
  LoopCandidate(uint64_t id1, uint64_t id2) : idx1(id1), idx2(id2) {}
  uint64_t idx1{0};
  uint64_t idx2{0};
  SE3d tij;
  double ndt_score{0.0};
};

struct BackendKeyframe {
  using Ptr = std::shared_ptr<BackendKeyframe>;
  long id{-1};
  double stamp{0.0};
  pcl::PointCloud<PointType>::Ptr cloud;
  SE3d lio_pose;
  SE3d opt_pose;
};

// MIAO style backend: subscribes frontend keyframes, runs distance-gated
// loop detection with multi-resolution NDT verification and incremental
// sparse pose-graph optimization, and emits the optimized trajectory.
class OptimizationMIAO : public ExtensionModule {
public:
  OptimizationMIAO();
  ~OptimizationMIAO() override;
  void stop() override;

private:
  void on_new_frame(const KeyFrame::ConstPtr& frame);
  void input_thread();
  void publish_loop();

  void add_keyframe(long id, double stamp, const SE3d& lio_pose, const pcl::PointCloud<PointType>::Ptr& cloud);
  void handle_kf(const BackendKeyframe::Ptr& kf);
  void detect_loop_candidates();
  void compute_loop_candidates();
  void compute_for_candidate(LoopCandidate& c);
  void pose_optimization();
  pcl::PointCloud<PointType>::Ptr build_submap(int given_id, bool in_world) const;
  pcl::PointCloud<PointType>::Ptr voxel_grid(const pcl::PointCloud<PointType>::Ptr& cloud, float size) const;
  void worker_loop();
  bool get_latest_optimized_pose(double& stamp, SE3d& pose) const;

  struct InputFrame {
    long id{-1};
    double stamp{0.0};
    SE3d pose;
    PointCloudT::Ptr cloud{nullptr};
  };

  int on_new_frame_id{-1};

  static constexpr std::size_t queue_capacity = 50;

  // The cloud copy and keyframe hand-off run on the input thread; the
  // on_new_frame callback only enqueues, so the odometry worker is never
  // blocked by the backend.
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<InputFrame> queue{queue_capacity};
  std::size_t dropped_items{0};

  bool verbose{true};
  int loop_kf_gap{20};
  int min_id_interval{20};
  int closest_id_th{50};
  double max_range{30.0};
  double ndt_score_th{1.0};
  double motion_trans_noise{0.1};
  double motion_rot_noise{3.0 * M_PI / 180.0};
  double loop_trans_noise{0.2};
  double loop_rot_noise{3.0 * M_PI / 180.0};
  double rk_loop_th{5.2 / 5.0};
  bool with_height{true};
  double height_noise{0.1};

  int submap_idx_range{40};

  std::shared_ptr<::lightning::miao::Optimizer> optimizer{nullptr};
  Mat6d info_motion{Mat6d::Identity()};
  Mat6d info_loops{Mat6d::Identity()};

  std::vector<BackendKeyframe::Ptr> all_keyframes;
  BackendKeyframe::Ptr last_kf{nullptr};
  BackendKeyframe::Ptr last_loop_kf{nullptr};
  BackendKeyframe::Ptr cur_kf{nullptr};
  std::vector<LoopCandidate> candidates;

  std::vector<std::shared_ptr<::lightning::miao::VertexSE3>> kf_vert;
  std::vector<std::shared_ptr<::lightning::miao::EdgeSE3>> edge_loops;

  std::queue<BackendKeyframe::Ptr> kf_queue;
  mutable std::mutex mtx_queue;
  mutable std::mutex state_mutex;

  double publish_interval{0.1};
  std::atomic<bool> kill_switch{false};
  std::thread input_thread_obj;
  std::thread publish_thread;
  std::thread worker_thread;

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace optimization_miao
}  // namespace asuka
