#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <asuka/odometry/superlio/alias.hpp>

namespace asuka {
namespace superlio {

class So3 {
public:
  So3(const M3& R = Eye3);
  So3(const V3& w);
  So3(const So3& R);

  template <typename OtherDerived>
  So3(const Eigen::MatrixBase<OtherDerived>& rhs) : r(rhs) {}

  static M3 hat(const V3& v);
  static V3 vee(const M3& w_hat);
  static So3 exp(const V3& ang);
  static So3 exp(const V3& ang_vel, const Scalar& dt);
  static M3 exp_m3(const V3& ang_vel, const Scalar& dt);

  So3& operator=(const So3& rhs);
  So3 operator*(const So3& rhs) const noexcept;
  So3 operator*(Scalar s) const noexcept;
  V3 operator*(const V3& rhs) const noexcept;

  template <typename OtherDerived>
  V3 operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const noexcept {
    return r * rhs;
  }

  void update(const V3& dw);
  void update_rhs(const V3& dw);
  void apply_exp(const M3& w_hat);

  V4 coeffs() const noexcept;
  M3 log(double* ro = nullptr) const noexcept;
  V3 log_vee() const noexcept;
  So3 inverse() const noexcept;
  M3 adjoint() const noexcept;
  Quat quaternion() const noexcept;
  Scalar distance(const So3& rhs) const noexcept;
  Scalar yaw() const noexcept;

public:
  M3 r;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Se3 {
public:
  Se3() : mat(M4::Identity()), r(M3::Identity()), t(V3::Zero()) {}
  Se3(const M4& T);
  Se3(const V6& xi);
  Se3(const Se3& T);
  Se3(const So3& R, const V3& t);
  Se3(const M3& R, const V3& t);
  Se3(const Quat& q, const V3& t);

  template <typename OtherDerived>
  Se3(const Eigen::MatrixBase<OtherDerived>& rhs) : mat(rhs) {}

  Se3& operator=(const Se3& rhs);
  Se3 operator*(const Se3& rhs) const noexcept;
  V3 operator*(const V3& point) const noexcept;
  V4 operator*(const V4& point) const noexcept;

  void update(const V6& dxi);
  void update_rhs(const V6& dxi);
  void apply_exp(const M4& xi_hat);
  M4 log() const noexcept;
  V6 log_vee() const noexcept;
  V3 transform(const V3& p) const noexcept;
  Se3 inverse() const noexcept;
  M6 adjoint() const noexcept;
  Quat quaternion() const noexcept;
  double distance(const Se3& rhs) const noexcept;

public:
  M4 mat;
  M3 r;
  V3 t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

M4 hat6(const V6& xi);
V6 vee6(const M4& xi_hat);

}  // namespace superlio
}  // namespace asuka
