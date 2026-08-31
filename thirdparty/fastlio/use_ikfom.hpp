#pragma once

#include <fastlio/mtk/build_manifold.hpp>
#include <fastlio/mtk/startIdx.hpp>
#include <fastlio/mtk/types/S2.hpp>
#include <fastlio/mtk/types/SOn.hpp>
#include <fastlio/mtk/types/vect.hpp>

namespace ikfom {

using vect3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;
using S2 = MTK::S2<double, 98090, 10000, 1>;
using vect1 = MTK::vect<1, double>;
using vect2 = MTK::vect<2, double>;

/**
 * @brief Manifold state for the iterated error-state Kalman filter.
 *
 * Mirrors fastlio's `state_ikfom`. The order of fields matters because the IESKF update
 * relies on the compiled SO3/S2 index bookkeeping done by the MTK toolkit.
 */
MTK_BUILD_MANIFOLD(
  state_ikfom,
  ((vect3,
    pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba))((S2, grav)))

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)))

MTK_BUILD_MANIFOLD(process_noise_ikfom, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)))

/**
 * @brief Default process noise covariance for the IESKF (12-DOF).
 */
inline MTK::get_cov<process_noise_ikfom>::type process_noise_cov() {
  MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
  MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);
  MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);
  MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001);
  MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);
  return cov;
}

/**
 * @brief Continuous-time process model f(s, in) used by the IESKF prediction step.
 */
inline Eigen::Matrix<double, 24, 1> get_f(state_ikfom& s, const input_ikfom& in) {
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.grav[i];
  }
  return res;
}

/**
 * @brief Jacobian of f w.r.t. the state (df/dx).
 */
inline Eigen::Matrix<double, 24, 23> df_dx(state_ikfom& s, const input_ikfom& in) {
  Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_relative;
  in.acc.boxminus(acc_relative, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_relative);
  cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
  Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 2>(12, 21) = grav_matrix;
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

/**
 * @brief Jacobian of f w.r.t. the process noise (df/dw).
 */
inline Eigen::Matrix<double, 24, 12> df_dw(state_ikfom& s, const input_ikfom& in) {
  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
  cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return cov;
}

/**
 * @brief Convert an SO3 quaternion to Euler angles (roll, pitch, yaw) in degrees.
 */
inline vect3 SO3ToEuler(const SO3& orient) {
  Eigen::Matrix<double, 3, 1> ang;
  Eigen::Vector4d q_data = orient.coeffs().transpose();
  double sqw = q_data[3] * q_data[3];
  double sqx = q_data[0] * q_data[0];
  double sqy = q_data[1] * q_data[1];
  double sqz = q_data[2] * q_data[2];
  double unit = sqx + sqy + sqz + sqw;
  double test = q_data[3] * q_data[1] - q_data[2] * q_data[0];

  if (test > 0.49999 * unit) {
    ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI / 2, 0;
    double temp[3] = {ang[0] * 57.3, ang[1] * 57.3, ang[2] * 57.3};
    return vect3(temp, 3);
  }
  if (test < -0.49999 * unit) {
    ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI / 2, 0;
    double temp[3] = {ang[0] * 57.3, ang[1] * 57.3, ang[2] * 57.3};
    return vect3(temp, 3);
  }

  ang << std::atan2(2 * q_data[0] * q_data[3] + 2 * q_data[1] * q_data[2], -sqx - sqy + sqz + sqw),
    std::asin(2 * test / unit),
    std::atan2(2 * q_data[2] * q_data[3] + 2 * q_data[1] * q_data[0], sqx - sqy - sqz + sqw);
  double temp[3] = {ang[0] * 57.3, ang[1] * 57.3, ang[2] * 57.3};
  return vect3(temp, 3);
}

}  // namespace ikfom
