#pragma once
// Ported from sr_livo include/rgbMapTracker.h. The upstream file-scope
// globals (g_voxel_for_render, render_point_count) became class members.

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <asuka/modules/srlivo/cloud_map.hpp>
#include <asuka/modules/srlivo/frame.hpp>
#include <asuka/modules/srlivo/state.hpp>
#include <asuka/modules/srlivo/utility.hpp>

namespace asuka {
namespace srlivo {

class rgbMapTracker {
public:

  std::vector<voxelId> voxels_recent_visited;

  std::vector<rgbPoint *> rgb_points_vec;
  std::vector<rgbPoint *> *points_rgb_vec_for_projection;

  std::shared_ptr<std::mutex> mutex_rgb_points_vec;

  cloudFrame *p_cloud_frame;

  double minimum_depth_for_projection;
  double maximum_depth_for_projection;

  double recent_visited_voxel_activated_time;

  int number_of_new_visited_voxel;

  int updated_frame_index;

  std::shared_ptr<std::mutex> mutex_frame_index;

  bool in_appending_points;

  rgbMapTracker();
  ~rgbMapTracker();

  void threadRenderPointsInVoxel(voxelHashMap &map, const int &voxel_start, const int &voxel_end, cloudFrame *p_frame,
                                 const std::vector<voxelId> *voxels_for_render, const double obs_time);

  void renderPointsInRecentVoxel(voxelHashMap &map, cloudFrame *p_frame, std::vector<voxelId> *voxels_for_render, const double &obs_time);

  void refreshPointsForProjection(voxelHashMap &map);

  void selectPointsForProjection(voxelHashMap &map, cloudFrame *p_frame, std::vector<rgbPoint *> *pc_out_vec = nullptr, std::vector<cv::Point2f> *pc_2d_out_vec = nullptr,
                                 double minimum_dis = 5, int skip_step = 1, bool use_all_points = false);

  void updatePoseForProjection(cloudFrame *p_frame, double fov_margin = 0.0001);

private:
  std::vector<voxelId> voxel_for_render;
  std::atomic<long> render_point_count{0};
};

}  // namespace srlivo
}  // namespace asuka
