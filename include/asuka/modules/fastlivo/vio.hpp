/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include <asuka/modules/fastlivo/voxel_map.hpp>
#include <asuka/modules/fastlivo/feature.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

namespace asuka {
namespace fastlivo {

struct SubSparseMap
{
  std::vector<float> propa_errors;
  std::vector<float> errors;
  std::vector<std::vector<float>> warp_patch;
  std::vector<int> search_levels;
  std::vector<VisualPoint *> voxel_points;
  std::vector<double> inv_expo_list;
  std::vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Eigen::Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Eigen::Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  Eigen::Matrix3d Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  Eigen::Vector3d Pli, Pci, Pcl, Pcw;
  std::vector<int> grid_num;
  std::vector<int> map_index;
  std::vector<int> border_flag;
  std::vector<int> update_flag;
  std::vector<float> map_dist;
  std::vector<float> scan_value;
  std::vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<Eigen::Vector3d>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  std::unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  std::unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  std::unordered_map<int, Warp *> warp_map;
  std::vector<VisualPoint *> retrieve_voxel_points;
  std::vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  // Latest camera frame in BGR (shallow cv::Mat handle); used to
  // colorize the accumulated map after each VIO update.
  cv::Mat img_rgb;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, std::vector<pointWithVar> &pg, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, std::vector<pointWithVar> &pg, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, std::vector<pointWithVar> &pg);
  void setImuToLidarExtrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot);
  void setLidarToCameraExtrinsic(std::vector<double> &R, std::vector<double> &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, Eigen::Vector2d pc, float *patch_tmp, int level);
  void computeProjectionJacobian(Eigen::Vector3d p, Eigen::Matrix<double, 2, 3> & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Eigen::Vector2d &px_ref, const Eigen::Vector3d &f_ref, const double depth_ref, const Sophus::SE3d &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Eigen::Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const Eigen::Vector2d &px_ref,
                                     const Eigen::Vector3d &xyz_ref, const Eigen::Vector3d &normal_ref, const Sophus::SE3d &T_cur_ref, const int level_ref, Eigen::Matrix2d &A_cur_ref);
  void warpAffine(const Eigen::Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Eigen::Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void updateFrameState(StatesGroup state);
  void updateReferencePatch(const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Eigen::Matrix2d &A_cur_ref, const int max_level);
  Eigen::Vector3f getInterpolatedPixel(cv::Mat img, Eigen::Vector2d pc);
  
  // void resetRvizDisplay();
  // std::deque<VisualPoint *> map_cur_frame;
  // std::deque<VisualPoint *> sub_map_ray;
  // std::deque<VisualPoint *> sub_map_ray_fov;
  // std::deque<VisualPoint *> visual_sub_map_cur;
  // std::deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<Eigen::Vector3d>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

}  // namespace fastlivo
}  // namespace asuka

#endif // VIO_H_