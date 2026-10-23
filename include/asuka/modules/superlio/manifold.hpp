#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <asuka/modules/superlio/alias.hpp>

namespace asuka {
namespace superlio {

class So3 {
public:
  So3(const Eigen::Matrix<float, 3, 3>& R = Eye3);
  So3(const Eigen::Matrix<float, 3, 1>& w);
  So3(const So3& R);

  template <typename OtherDerived>
  So3(const Eigen::MatrixBase<OtherDerived>& rhs) : r(rhs) {}

  static Eigen::Matrix<float, 3, 3> hat(const Eigen::Matrix<float, 3, 1>& v);
  static Eigen::Matrix<float, 3, 1> vee(const Eigen::Matrix<float, 3, 3>& w_hat);
  static So3 exp(const Eigen::Matrix<float, 3, 1>& ang);
  static So3 exp(const Eigen::Matrix<float, 3, 1>& ang_vel, const float& dt);
  static Eigen::Matrix<float, 3, 3> exp_m3(const Eigen::Matrix<float, 3, 1>& ang_vel, const float& dt);

  So3& operator=(const So3& rhs);
  So3 operator*(const So3& rhs) const noexcept;
  So3 operator*(float s) const noexcept;
  Eigen::Matrix<float, 3, 1> operator*(const Eigen::Matrix<float, 3, 1>& rhs) const noexcept;

  template <typename OtherDerived>
  Eigen::Matrix<float, 3, 1> operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const noexcept {
    return r * rhs;
  }

  void update(const Eigen::Matrix<float, 3, 1>& dw);
  void update_rhs(const Eigen::Matrix<float, 3, 1>& dw);
  void apply_exp(const Eigen::Matrix<float, 3, 3>& w_hat);

  Eigen::Matrix<float, 4, 1> coeffs() const noexcept;
  Eigen::Matrix<float, 3, 3> log(double* ro = nullptr) const noexcept;
  Eigen::Matrix<float, 3, 1> log_vee() const noexcept;
  So3 inverse() const noexcept;
  Eigen::Matrix<float, 3, 3> adjoint() const noexcept;
  Eigen::Quaternion<float> quaternion() const noexcept;
  float distance(const So3& rhs) const noexcept;
  float yaw() const noexcept;

public:
  Eigen::Matrix<float, 3, 3> r;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Se3 {
public:
  Se3() : mat(Eigen::Matrix<float, 4, 4>::Identity()), r(Eigen::Matrix<float, 3, 3>::Identity()), t(Eigen::Matrix<float, 3, 1>::Zero()) {}
  Se3(const Eigen::Matrix<float, 4, 4>& T);
  Se3(const Eigen::Matrix<float, 6, 1>& xi);
  Se3(const Se3& T);
  Se3(const So3& R, const Eigen::Matrix<float, 3, 1>& t);
  Se3(const Eigen::Matrix<float, 3, 3>& R, const Eigen::Matrix<float, 3, 1>& t);
  Se3(const Eigen::Quaternion<float>& q, const Eigen::Matrix<float, 3, 1>& t);

  template <typename OtherDerived>
  Se3(const Eigen::MatrixBase<OtherDerived>& rhs) : mat(rhs) {}

  Se3& operator=(const Se3& rhs);
  Se3 operator*(const Se3& rhs) const noexcept;
  Eigen::Matrix<float, 3, 1> operator*(const Eigen::Matrix<float, 3, 1>& point) const noexcept;
  Eigen::Matrix<float, 4, 1> operator*(const Eigen::Matrix<float, 4, 1>& point) const noexcept;

  void update(const Eigen::Matrix<float, 6, 1>& dxi);
  void update_rhs(const Eigen::Matrix<float, 6, 1>& dxi);
  void apply_exp(const Eigen::Matrix<float, 4, 4>& xi_hat);
  Eigen::Matrix<float, 4, 4> log() const noexcept;
  Eigen::Matrix<float, 6, 1> log_vee() const noexcept;
  Eigen::Matrix<float, 3, 1> transform(const Eigen::Matrix<float, 3, 1>& p) const noexcept;
  Se3 inverse() const noexcept;
  Eigen::Matrix<float, 6, 6> adjoint() const noexcept;
  Eigen::Quaternion<float> quaternion() const noexcept;
  double distance(const Se3& rhs) const noexcept;

public:
  Eigen::Matrix<float, 4, 4> mat;
  Eigen::Matrix<float, 3, 3> r;
  Eigen::Matrix<float, 3, 1> t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Matrix<float, 4, 4> hat6(const Eigen::Matrix<float, 6, 1>& xi);
Eigen::Matrix<float, 6, 1> vee6(const Eigen::Matrix<float, 4, 4>& xi_hat);

}  // namespace superlio
}  // namespace asuka
