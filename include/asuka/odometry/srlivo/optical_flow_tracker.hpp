#pragma once
// Ported from sr_livo include/opticalFlowTracker.h. The ANSI-color macros and
// scope_color helper are not migrated (logging goes through spdlog).

#include <map>
#include <math.h>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>

#include <asuka/odometry/srlivo/cloud_map.hpp>
#include <asuka/odometry/srlivo/frame.hpp>
#include <asuka/odometry/srlivo/lk_pyramid.hpp>
#include <asuka/odometry/srlivo/utility.hpp>

namespace asuka {
namespace srlivo {

class cloudFrame;
class rgbMapTracker;
class opticalFlowTracker {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  cv::Mat old_image, old_gray;
  cv::Mat cur_image, cur_gray;
  cv::Mat cur_mask;
  unsigned int image_idx = 0;
  double last_image_time, current_image_time;
  std::vector<int> cur_ids, old_ids;
  unsigned int maximum_tracked_points;
  cv::Mat m_ud_map1, m_ud_map2;
  cv::Mat intrinsic, dist_coeffs;
  std::vector<cv::Point2f> last_tracked_points, cur_tracked_points;
  std::vector<void *> rgb_points_ptr_vec_in_last_image;
  std::map<void *, cv::Point2f> map_rgb_points_in_last_image_pose;
  std::map<void *, cv::Point2f> map_rgb_points_in_cur_image_pose;
  std::map<int, std::vector<cv::Point2f>> map_id_points_vec;
  std::map<int, std::vector<int>> map_id_points_image;
  std::map<int, std::vector<cv::Point2f>> map_image_points;
  Eigen::Quaterniond last_quat;
  Eigen::Vector3d last_trans;
  std::shared_ptr<LKOpticalFlowKernel> lk_optical_flow_kernel;
  opticalFlowTracker();
  void setIntrinsic(Eigen::Matrix3d intrinsic_, Eigen::Matrix<double, 5, 1> dist_coeffs_, cv::Size image_size_);
  void trackImage(cloudFrame *p_frame, double distance);
  void init(cloudFrame *p_frame, std::vector<rgbPoint *> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec);
  void setTrackPoints(cv::Mat &image, std::vector<rgbPoint *> &rgb_points_vec, std::vector<cv::Point2f> &points_2d_vec);
  void rejectErrorTrackingPoints(cloudFrame *p_frame, double distance);
  void updateAndAppendTrackPoints(cloudFrame *p_frame, rgbMapTracker *map_tracker, double mini_distance, int minimum_frame_diff);
  void updateLastTrackingVectorAndIds();
  bool removeOutlierUsingRansacPnp(cloudFrame *p_frame, int if_remove_ourlier = 1);

  template <typename T>
  static void reduce_vector(std::vector<T> &v, std::vector<uchar> status) {
    int j = 0;
    for (unsigned int i = 0; i < v.size(); i++)
      if (status[i]) v[j++] = v[i];
    v.resize(j);
  }
};

}  // namespace srlivo
}  // namespace asuka
