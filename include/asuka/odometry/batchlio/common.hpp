#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <esekfom/esekfom.hpp>
#include <faster_ivox/ivox3d.h>

#include <asuka/core/types.hpp>
#include <asuka/odometry/batchlio/so3.hpp>

namespace asuka {
namespace batch {

// ---- manifold scalars (common_lib.h) ----
typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(
  state_input,
  ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba))(
    (vect3, gravity)))

MTK_BUILD_MANIFOLD(
  state_output,
  ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, omg))((vect3, acc))(
    (vect3, gravity))((vect3, bg))((vect3, ba)))

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)))

MTK_BUILD_MANIFOLD(process_noise_input, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)))

MTK_BUILD_MANIFOLD(process_noise_output, ((vect3, vel))((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)))

// ---- point types ----
using PointType = PointT;
using PointCloudXYZI = PointCloudT;
using PointVector = PointVectorT;

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;
using M3F = Eigen::Matrix3f;

// upstream macros (common_lib.h)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define NUM_MATCH_POINTS (5)
#define MAX_INI_COUNT (100)
#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

constexpr int DIM_STATE = 24;
constexpr int DIM_PROC_N = 12;

// ---- IVox local map ----
using IVoxType = fasterlio::IVox<3, fasterlio::IVoxNodeType::DEFAULT, PointT>;

// ---- IMU/lidar bundle produced by OdometryBatchLIO::synchronize() ----
struct MeasureGroup {
  double lidar_begin_time{0.0};
  double lidar_end_time{0.0};
  double lidar_last_time{0.0};
  PointCloudT::ConstPtr lidar;
  std::vector<ImuData::ConstPtr> imu;
  std::vector<ImuData::ConstPtr> imu_initialization;
  ImuData::ConstPtr imu_after_end;
};

// ---- stateless EKF process models (Estimator.cpp) ----
Eigen::Matrix<double, 24, 1> get_f_input(state_input& s, const input_ikfom& in);
Eigen::Matrix<double, 30, 1> get_f_output(state_output& s, const input_ikfom& in);
Eigen::Matrix<double, 24, 24> df_dx_input(state_input& s, const input_ikfom& in);
Eigen::Matrix<double, 30, 30> df_dx_output(state_output& s, const input_ikfom& in);

// ---- time windowing / plane fitting (common_lib.h) ----
std::vector<int> time_compressing(const PointCloudT::Ptr& cloud);
std::vector<int> time_compressing_batch(const PointCloudT::Ptr& cloud, double win_ms);

template <typename T>
bool estimate_norm_vector(
  Eigen::Matrix<T, 3, 1>& norm_vector,
  const PointVectorT& point,
  const T& threshold,
  int point_num) {
  Eigen::Matrix<T, Eigen::Dynamic, 3> A(point_num, 3);
  Eigen::Matrix<T, Eigen::Dynamic, 1> b(point_num, 1);
  b.setOnes();
  b *= static_cast<T>(-1.0);
  for (int j = 0; j < point_num; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }
  norm_vector = A.colPivHouseholderQr().solve(b);
  for (int j = 0; j < point_num; j++) {
    if (
      std::fabs(
        norm_vector(0) * point[j].x + norm_vector(1) * point[j].y + norm_vector(2) * point[j].z + static_cast<T>(1.0)) >
      threshold) {
      return false;
    }
  }
  norm_vector.normalize();
  return true;
}

template <typename T>
bool estimate_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVectorT& point, const T& threshold) {
  Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= static_cast<T>(-1.0);
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }
  Eigen::Matrix<T, 3, 1> norm_vector = A.colPivHouseholderQr().solve(b);
  const T n = norm_vector.norm();
  pca_result(0) = norm_vector(0) / n;
  pca_result(1) = norm_vector(1) / n;
  pca_result(2) = norm_vector(2) / n;
  pca_result(3) = static_cast<T>(1.0) / n;
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (
      std::fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) >
      threshold) {
      return false;
    }
  }
  return true;
}

// ---- utility (parameters.cpp) ----
Eigen::Matrix<double, 3, 1> so3_to_euler(const SO3& orient);
void reset_cov(Eigen::Matrix<double, 24, 24>& p_init);
void reset_cov_output(Eigen::Matrix<double, 30, 30>& p_init_output);
bool time_list(PointT& x, PointT& y);

}  // namespace batch
}  // namespace asuka
