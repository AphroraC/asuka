#include <asuka/odometry/batchlio/common.hpp>

namespace asuka {
namespace batch {

Eigen::Matrix<double, 24, 1> get_f_input(state_input& s, const input_ikfom& in) {
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 30, 1> get_f_output(state_output& s, const input_ikfom& in) {
  Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
  vect3 a_inertial = s.rot * s.acc;
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = s.omg[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 24, 24> df_dx_input(state_input& s, const input_ikfom& in) {
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acceleration;
  in.acc.boxminus(acceleration, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acceleration);
  cov.template block<3, 3>(12, 18) = -s.rot;
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, 30, 30> df_dx_output(state_output& s, const input_ikfom& in) {
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(s.acc);
  cov.template block<3, 3>(12, 18) = s.rot;
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  return cov;
}

std::vector<int> time_compressing(const PointCloudT::Ptr& cloud) {
  const int points_size = static_cast<int>(cloud->points.size());
  int j = 0;
  std::vector<int> time_seq_local;
  time_seq_local.reserve(points_size);
  for (int i = 0; i < points_size - 1; i++) {
    j++;
    if (cloud->points[i + 1].curvature > cloud->points[i].curvature) {
      time_seq_local.emplace_back(j);
      j = 0;
    }
  }
  time_seq_local.emplace_back(j + 1);
  return time_seq_local;
}

std::vector<int> time_compressing_batch(const PointCloudT::Ptr& cloud, double win_ms) {
  const int points_size = static_cast<int>(cloud->points.size());
  std::vector<int> time_seq_local;
  if (points_size == 0) return time_seq_local;
  if (win_ms <= 0.0) return time_compressing(cloud);
  time_seq_local.reserve(points_size);
  int count = 1;
  long cur_win = static_cast<long>(cloud->points[0].curvature / win_ms);
  for (int i = 1; i < points_size; i++) {
    const long w = static_cast<long>(cloud->points[i].curvature / win_ms);
    if (w == cur_win) {
      count++;
    } else {
      time_seq_local.emplace_back(count);
      count = 1;
      cur_win = w;
    }
  }
  time_seq_local.emplace_back(count);
  return time_seq_local;
}

Eigen::Matrix<double, 3, 1> so3_to_euler(const SO3& rot) {
  const double sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  const bool singular = sy < 1e-6;
  double x;
  double y;
  double z;
  if (!singular) {
    x = std::atan2(rot(2, 1), rot(2, 2));
    y = std::atan2(-rot(2, 0), sy);
    z = std::atan2(rot(1, 0), rot(0, 0));
  } else {
    x = std::atan2(-rot(1, 2), rot(1, 1));
    y = std::atan2(-rot(2, 0), sy);
    z = 0;
  }
  return Eigen::Matrix<double, 3, 1>(x, y, z);
}

void reset_cov(Eigen::Matrix<double, 24, 24>& p_init) {
  p_init = MD(24, 24)::Identity() * 0.1;
  p_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  p_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30>& p_init_output) {
  p_init_output = MD(30, 30)::Identity() * 0.01;
  p_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  p_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}

bool time_list(PointT& x, PointT& y) {
  return (x.curvature < y.curvature);
}

}  // namespace batch
}  // namespace asuka
