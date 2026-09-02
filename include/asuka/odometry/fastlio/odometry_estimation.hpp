#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <omp.h>

#include <fastlio/pose6d.hpp>
#include <fastlio/so3_math.hpp>
#include <fastlio/use_ikfom.hpp>
#include <fastlio/esekfom/esekfom.hpp>
#include <ikd_tree/ikd_tree.hpp>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/fastlio/cloud_preprocess.hpp>
#include <asuka/odometry/fastlio/imu_preprocess.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlio {

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
  void lasermap_fov_segment();
  void point_body_to_world(const PointT& pi, PointT& po);
  void point_body_lidar_to_imu(const PointT& pi, PointT& po);
  static void h_share_model_trampoline(ikfom::state_ikfom& s, esekfom::dyn_share_datastruct<double>& data);
  void h_share_model(ikfom::state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);
  void map_incremental();

  template <typename T>
  bool estimate_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVectorT& point, const T threshold) {
    Eigen::Matrix<T, num_match_points, 3> A;
    Eigen::Matrix<T, num_match_points, 1> b;
    b.setConstant(-1.0);
    for (int j = 0; j < num_match_points; ++j) {
      A(j, 0) = point[j].x;
      A(j, 1) = point[j].y;
      A(j, 2) = point[j].z;
    }
    const Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    const T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;
    for (int j = 0; j < num_match_points; ++j) {
      if (
        std::fabs(
          pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) >
        threshold) {
        return false;
      }
    }
    return true;
  }

  static float calc_dist(const PointT& p1, const PointT& p2);

private:
  static constexpr double init_time = 0.1;
  static constexpr int num_match_points = 5;
  static constexpr float mov_threshold = 1.5f;
  static constexpr float lidar_sp_len = 2.0f;

  static std::vector<float>& tls_search_sq_dis() {
    static thread_local std::vector<float> buf;
    return buf;
  }

  double det_range{300.0};
  double fov_degree{180.0};

  int max_iterations{4};
  double laser_point_cov{0.001};

  bool save_map_en{false};

  double filter_size_surf{0.5};
  double filter_size_map{0.5};
  double cube_len{200.0};

  static thread_local OdometryEstimation* active;

  std::mutex buffer_mutex;

  boost::circular_buffer<PointCloudT::ConstPtr> lidar_buffer;
  boost::circular_buffer<double> time_buffer;
  boost::circular_buffer<ImuData::ConstPtr> imu_buffer;

  bool lidar_pushed{false};
  double lidar_end_time{0.0};
  double last_timestamp_imu{-1.0};
  double lidar_mean_scantime{0.0};
  int scan_num{0};

  PointCloudT::ConstPtr pending_lidar{nullptr};
  double pending_lidar_beg_time{0.0};
  double pending_lidar_end_time{0.0};

  std::shared_ptr<ImuPreprocess> imu_preprocess{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom> kf;
  ikfom::state_ikfom state_point;
  Eigen::Vector3d pos_lid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d euler_cur{Eigen::Vector3d::Zero()};
  double epsi[23] = {0.001};
  bool first_scan_flag{true};
  bool ekf_inited_flag{false};
  double first_lidar_time{0.0};

  KD_TREE<PointT> ikdtree;
  BoxPointType local_map_points;
  bool localmap_initialized{false};
  std::vector<BoxPointType> cub_needrm;

  PointCloudT::Ptr map_cloud{new PointCloudT()};

  pcl::VoxelGrid<PointT> downsize_filter_surf;

  PointCloudT::Ptr feats_undistort{new PointCloudT()};
  PointCloudT::Ptr feats_down_body{new PointCloudT()};
  PointCloudT::Ptr feats_down_world{new PointCloudT()};
  PointCloudT::Ptr normvec{new PointCloudT(100000, 1)};
  PointCloudT::Ptr laser_cloud_ori{new PointCloudT(100000, 1)};
  PointCloudT::Ptr corr_normvect{new PointCloudT(100000, 1)};
  PointCloudT::Ptr cloud_world{new PointCloudT()};
  PointCloudT::Ptr cloud_body{new PointCloudT()};

  std::vector<PointVectorT> nearest_points;
  std::vector<bool> point_selected_surf;
  std::vector<float> res_last;

  int feats_down_size{0};
  int effct_feat_num{0};
  double total_residual{0.0};
  double res_mean_last{0.05};

  long frame_counter{0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace fastlio
}  // namespace asuka
