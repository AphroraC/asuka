#include "asuka/odometry/superlio/manifold.hpp"

namespace asuka {
namespace superlio {

So3::So3(const M3& R) : r(R) {}

So3::So3(const V3& w) : r(Eye3) {
  this->apply_exp(hat(w));
}

So3::So3(const So3& R) : r(R.r) {}

M3 So3::hat(const V3& v) {
  M3 m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

V3 So3::vee(const M3& w_hat) {
  V3 w;
  w << -w_hat(1, 2), w_hat(0, 2), -w_hat(0, 1);
  return w;
}

So3 So3::exp(const V3& ang) {
  Scalar norm = ang.norm();
  M3 I = Eye3;
  if (norm > 1e-10) {
    V3 r_ang = ang / norm;
    M3 K;
    K = hat(r_ang);
    return So3(I + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K);
  } else {
    return So3(I);
  }
}

M3 So3::exp_m3(const V3& ang_vel, const Scalar& dt) {
  static const Scalar coefficient[] = {1.0 / 6.0, 1.0 / 24.0, 1.0 / 120.0, 1.0 / 720.0};

  V3 phi = ang_vel * dt;
  Scalar squared_norm = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  Scalar norm = std::sqrt(squared_norm);
  Scalar A, B;
  if (squared_norm >= 1e-6) {
    A = sin(norm) / norm;
    B = (1.0 - cos(norm)) / squared_norm;
  } else {
    const Scalar t4 = squared_norm * squared_norm;
    A = 1.0 - (squared_norm * coefficient[0]) + (t4 * coefficient[2]);
    B = 0.5 - (squared_norm * coefficient[1]) + (t4 * coefficient[3]);
  }

  M3 K;
  K << 0.0, -phi[2], phi[1], phi[2], 0.0, -phi[0], -phi[1], phi[0], 0.0;
  return M3::Identity() + A * K + B * K * K;
}

So3 So3::exp(const V3& ang_vel, const Scalar& dt) {
  Scalar ang_vel_norm = ang_vel.norm();
  M3 I = Eye3;
  if (ang_vel_norm > 1e-12) {
    V3 r_axis = ang_vel / ang_vel_norm;
    M3 K;
    K = hat(r_axis);
    Scalar r_ang = ang_vel_norm * dt;
    return So3(I + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K);
  } else {
    return So3(I);
  }
}

So3& So3::operator=(const So3& rhs) {
  if (this == &rhs) return *this;
  r = rhs.r;
  return *this;
}

So3 So3::operator*(const So3& rhs) const noexcept {
  M3 res = r * rhs.r;
  return So3(res);
}

So3 So3::operator*(Scalar s) const noexcept {
  M3 res = r * s;
  return So3(res);
}

V3 So3::operator*(const V3& rhs) const noexcept {
  return r * rhs;
}

void So3::update(const V3& dw) {
  So3 dr(dw);
  r = dr.r * r;
}

void So3::update_rhs(const V3& dw) {
  So3 dr(dw);
  r = r * dr.r;
}

void So3::apply_exp(const M3& w_hat) {
  V3 w = vee(w_hat);
  double o = w.norm();
  if (o < 1e-12) {
    r << Eye3 + w_hat;
    return;
  }
  double c1 = std::sin(o) / o;
  double c2 = (1 - std::cos(o)) / o / o;
  r << Eye3 + c1 * w_hat + c2 * w_hat * w_hat;
}

V4 So3::coeffs() const noexcept {
  return Quat(r).coeffs();
}

M3 So3::log(double* ro) const noexcept {
  const Eigen::Matrix3d R_double = r.cast<double>();

  Eigen::Quaterniond q(R_double);
  q.normalize();

  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector_part(q.x(), q.y(), q.z());

  const double sin_half_theta = vector_part.norm();

  Eigen::Vector3d rotation_vector;
  double theta = 0.0;

  if (sin_half_theta < 1e-12) {
    rotation_vector = 2.0 * vector_part;
    theta = rotation_vector.norm();
  } else {
    theta = 2.0 * std::atan2(sin_half_theta, q.w());
    rotation_vector = theta / sin_half_theta * vector_part;
  }

  if (ro != nullptr) {
    *ro = theta;
  }

  return So3::hat(rotation_vector.cast<Scalar>());
}

V3 So3::log_vee() const noexcept {
  M3 w_hat = this->log();
  return vee(w_hat);
}

So3 So3::inverse() const noexcept {
  return So3(r.transpose());
}

M3 So3::adjoint() const noexcept {
  return r.transpose();
}

Quat So3::quaternion() const noexcept {
  Quat res = Quat(r);
  return res.normalized();
}

Scalar So3::distance(const So3& rhs) const noexcept {
  return (*this * rhs.inverse()).log_vee().norm();
}

Scalar So3::yaw() const noexcept {
  Scalar sy = sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
  if (sy < 1e-6) return 0;
  return atan2(r(1, 0), r(0, 0));
}

Se3::Se3(const M4& T) : mat(T) {
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

Se3::Se3(const V6& xi) {
  this->apply_exp(hat6(xi));
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

Se3::Se3(const Se3& T) : mat(T.mat), r(T.r), t(T.t) {}

Se3::Se3(const So3& R, const V3& t) : mat(M4::Identity()) {
  mat.block<3, 3>(0, 0) = R.r;
  mat.block<3, 1>(0, 3) = t;
  r = R.r;
  this->t = t;
}

Se3::Se3(const M3& R, const V3& t) : mat(M4::Identity()) {
  mat.block<3, 3>(0, 0) = R;
  mat.block<3, 1>(0, 3) = t;
  r = R;
  this->t = t;
}

Se3::Se3(const Quat& q, const V3& t) : mat(M4::Identity()) {
  mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  mat.block<3, 1>(0, 3) = t;
  r = mat.topLeftCorner<3, 3>();
  this->t = t;
}

Se3& Se3::operator=(const Se3& rhs) {
  if (this == &rhs) return *this;
  mat = rhs.mat;
  r = rhs.r;
  t = rhs.t;
  return *this;
}

Se3 Se3::operator*(const Se3& rhs) const noexcept {
  M4 res = mat * rhs.mat;
  return Se3(res);
}

V3 Se3::operator*(const V3& point) const noexcept {
  return r * point + t;
}

V4 Se3::operator*(const V4& point) const noexcept {
  return mat * point;
}

void Se3::update(const V6& dxi) {
  Se3 dt(dxi);
  mat = dt.mat * mat;
}

void Se3::update_rhs(const V6& dxi) {
  Se3 dt(dxi);
  mat = mat * dt.mat;
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

void Se3::apply_exp(const M4& xi_hat) {
  V6 xi = vee6(xi_hat);
  V3 w = xi.head<3>();
  V3 v = xi.tail<3>();
  So3 rotation(w);
  M3 w_hat = xi_hat.topLeftCorner<3, 3>();

  M3 V = M3::Identity();
  double o = w.norm();
  if (o > 1e-12) {
    double c2 = (1 - std::cos(o)) / o / o;
    double c3 = (o - std::sin(o)) / o / o / o;
    V += c2 * w_hat + c3 * w_hat * w_hat;
  }
  V3 tt = V * v;

  mat << rotation.r, tt, 0, 0, 0, 1;
  r = rotation.r;
  t = tt;
}

M4 Se3::log() const noexcept {
  So3 rotation(this->r);
  double o;
  M3 w_hat = rotation.log(&o);
  M3 Vinv = M3::Identity();
  if (o > 1e-12) {
    double c1 = std::sin(o);
    double c2 = (1 - std::cos(o)) / o;
    double k1 = 1 / o / o * (1 - 0.5 * c1 / c2);
    Vinv += -0.5 * w_hat + k1 * w_hat * w_hat;
  }
  V3 v = Vinv * mat.topRightCorner<3, 1>();

  M4 xi_hat = M4::Zero();
  xi_hat << w_hat, v, 0, 0, 0, 0;
  return xi_hat;
}

V6 Se3::log_vee() const noexcept {
  M4 xi_hat = this->log();
  return vee6(xi_hat);
}

V3 Se3::transform(const V3& p) const noexcept {
  return r * p + t;
}

Se3 Se3::inverse() const noexcept {
  M4 inv;
  M3 R = r;
  R.transposeInPlace();
  inv << R, -R * this->t, 0, 0, 0, 1;
  return Se3(inv);
}

M6 Se3::adjoint() const noexcept {
  M6 res(M6::Zero());
  M3 tx = So3::hat(this->t);
  res.topLeftCorner<3, 3>() << r;
  res.bottomRightCorner<3, 3>() << r;
  res.bottomLeftCorner<3, 3>() << tx * r;
  return res;
}

Quat Se3::quaternion() const noexcept {
  Quat res = Quat(r);
  return res.normalized();
}

double Se3::distance(const Se3& rhs) const noexcept {
  return (*this * rhs.inverse()).log_vee().norm();
}

M4 hat6(const V6& xi) {
  M4 xi_hat;
  xi_hat << 0.0, -xi(2), xi(1), xi(3), xi(2), 0.0, -xi(0), xi(4), -xi(1), xi(0), 0.0, xi(5), 0.0, 0.0, 0.0, 0.0;
  return xi_hat;
}

V6 vee6(const M4& xi_hat) {
  V6 xi;
  xi << -xi_hat(1, 2), xi_hat(0, 2), -xi_hat(0, 1), xi_hat(0, 3), xi_hat(1, 3), xi_hat(2, 3);
  return xi;
}

}  // namespace superlio
}  // namespace asuka
