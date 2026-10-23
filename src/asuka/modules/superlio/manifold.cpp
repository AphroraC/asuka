#include "asuka/modules/superlio/manifold.hpp"

namespace asuka {
namespace superlio {

So3::So3(const Eigen::Matrix<float, 3, 3>& R) : r(R) {}

So3::So3(const Eigen::Matrix<float, 3, 1>& w) : r(Eye3) {
  this->apply_exp(hat(w));
}

So3::So3(const So3& R) : r(R.r) {}

Eigen::Matrix<float, 3, 3> So3::hat(const Eigen::Matrix<float, 3, 1>& v) {
  Eigen::Matrix<float, 3, 3> m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

Eigen::Matrix<float, 3, 1> So3::vee(const Eigen::Matrix<float, 3, 3>& w_hat) {
  Eigen::Matrix<float, 3, 1> w;
  w << -w_hat(1, 2), w_hat(0, 2), -w_hat(0, 1);
  return w;
}

So3 So3::exp(const Eigen::Matrix<float, 3, 1>& ang) {
  float norm = ang.norm();
  Eigen::Matrix<float, 3, 3> I = Eye3;
  if (norm > 1e-10) {
    Eigen::Matrix<float, 3, 1> r_ang = ang / norm;
    Eigen::Matrix<float, 3, 3> K;
    K = hat(r_ang);
    return So3(I + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K);
  } else {
    return So3(I);
  }
}

Eigen::Matrix<float, 3, 3> So3::exp_m3(const Eigen::Matrix<float, 3, 1>& ang_vel, const float& dt) {
  static const float coefficient[] = {1.0 / 6.0, 1.0 / 24.0, 1.0 / 120.0, 1.0 / 720.0};

  Eigen::Matrix<float, 3, 1> phi = ang_vel * dt;
  float squared_norm = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  float norm = std::sqrt(squared_norm);
  float A, B;
  if (squared_norm >= 1e-6) {
    A = sin(norm) / norm;
    B = (1.0 - cos(norm)) / squared_norm;
  } else {
    const float t4 = squared_norm * squared_norm;
    A = 1.0 - (squared_norm * coefficient[0]) + (t4 * coefficient[2]);
    B = 0.5 - (squared_norm * coefficient[1]) + (t4 * coefficient[3]);
  }

  Eigen::Matrix<float, 3, 3> K;
  K << 0.0, -phi[2], phi[1], phi[2], 0.0, -phi[0], -phi[1], phi[0], 0.0;
  return Eigen::Matrix<float, 3, 3>::Identity() + A * K + B * K * K;
}

So3 So3::exp(const Eigen::Matrix<float, 3, 1>& ang_vel, const float& dt) {
  float ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<float, 3, 3> I = Eye3;
  if (ang_vel_norm > 1e-12) {
    Eigen::Matrix<float, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<float, 3, 3> K;
    K = hat(r_axis);
    float r_ang = ang_vel_norm * dt;
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
  Eigen::Matrix<float, 3, 3> res = r * rhs.r;
  return So3(res);
}

So3 So3::operator*(float s) const noexcept {
  Eigen::Matrix<float, 3, 3> res = r * s;
  return So3(res);
}

Eigen::Matrix<float, 3, 1> So3::operator*(const Eigen::Matrix<float, 3, 1>& rhs) const noexcept {
  return r * rhs;
}

void So3::update(const Eigen::Matrix<float, 3, 1>& dw) {
  So3 dr(dw);
  r = dr.r * r;
}

void So3::update_rhs(const Eigen::Matrix<float, 3, 1>& dw) {
  So3 dr(dw);
  r = r * dr.r;
}

void So3::apply_exp(const Eigen::Matrix<float, 3, 3>& w_hat) {
  Eigen::Matrix<float, 3, 1> w = vee(w_hat);
  double o = w.norm();
  if (o < 1e-12) {
    r << Eye3 + w_hat;
    return;
  }
  double c1 = std::sin(o) / o;
  double c2 = (1 - std::cos(o)) / o / o;
  r << Eye3 + c1 * w_hat + c2 * w_hat * w_hat;
}

Eigen::Matrix<float, 4, 1> So3::coeffs() const noexcept {
  return Eigen::Quaternion<float>(r).coeffs();
}

Eigen::Matrix<float, 3, 3> So3::log(double* ro) const noexcept {
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

  return So3::hat(rotation_vector.cast<float>());
}

Eigen::Matrix<float, 3, 1> So3::log_vee() const noexcept {
  Eigen::Matrix<float, 3, 3> w_hat = this->log();
  return vee(w_hat);
}

So3 So3::inverse() const noexcept {
  return So3(r.transpose());
}

Eigen::Matrix<float, 3, 3> So3::adjoint() const noexcept {
  return r.transpose();
}

Eigen::Quaternion<float> So3::quaternion() const noexcept {
  Eigen::Quaternion<float> res = Eigen::Quaternion<float>(r);
  return res.normalized();
}

float So3::distance(const So3& rhs) const noexcept {
  return (*this * rhs.inverse()).log_vee().norm();
}

float So3::yaw() const noexcept {
  float sy = sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
  if (sy < 1e-6) return 0;
  return atan2(r(1, 0), r(0, 0));
}

Se3::Se3(const Eigen::Matrix<float, 4, 4>& T) : mat(T) {
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

Se3::Se3(const Eigen::Matrix<float, 6, 1>& xi) {
  this->apply_exp(hat6(xi));
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

Se3::Se3(const Se3& T) : mat(T.mat), r(T.r), t(T.t) {}

Se3::Se3(const So3& R, const Eigen::Matrix<float, 3, 1>& t) : mat(Eigen::Matrix<float, 4, 4>::Identity()) {
  mat.block<3, 3>(0, 0) = R.r;
  mat.block<3, 1>(0, 3) = t;
  r = R.r;
  this->t = t;
}

Se3::Se3(const Eigen::Matrix<float, 3, 3>& R, const Eigen::Matrix<float, 3, 1>& t) : mat(Eigen::Matrix<float, 4, 4>::Identity()) {
  mat.block<3, 3>(0, 0) = R;
  mat.block<3, 1>(0, 3) = t;
  r = R;
  this->t = t;
}

Se3::Se3(const Eigen::Quaternion<float>& q, const Eigen::Matrix<float, 3, 1>& t) : mat(Eigen::Matrix<float, 4, 4>::Identity()) {
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
  Eigen::Matrix<float, 4, 4> res = mat * rhs.mat;
  return Se3(res);
}

Eigen::Matrix<float, 3, 1> Se3::operator*(const Eigen::Matrix<float, 3, 1>& point) const noexcept {
  return r * point + t;
}

Eigen::Matrix<float, 4, 1> Se3::operator*(const Eigen::Matrix<float, 4, 1>& point) const noexcept {
  return mat * point;
}

void Se3::update(const Eigen::Matrix<float, 6, 1>& dxi) {
  Se3 dt(dxi);
  mat = dt.mat * mat;
}

void Se3::update_rhs(const Eigen::Matrix<float, 6, 1>& dxi) {
  Se3 dt(dxi);
  mat = mat * dt.mat;
  r = mat.topLeftCorner<3, 3>();
  t = mat.topRightCorner<3, 1>();
}

void Se3::apply_exp(const Eigen::Matrix<float, 4, 4>& xi_hat) {
  Eigen::Matrix<float, 6, 1> xi = vee6(xi_hat);
  Eigen::Matrix<float, 3, 1> w = xi.head<3>();
  Eigen::Matrix<float, 3, 1> v = xi.tail<3>();
  So3 rotation(w);
  Eigen::Matrix<float, 3, 3> w_hat = xi_hat.topLeftCorner<3, 3>();

  Eigen::Matrix<float, 3, 3> V = Eigen::Matrix<float, 3, 3>::Identity();
  double o = w.norm();
  if (o > 1e-12) {
    double c2 = (1 - std::cos(o)) / o / o;
    double c3 = (o - std::sin(o)) / o / o / o;
    V += c2 * w_hat + c3 * w_hat * w_hat;
  }
  Eigen::Matrix<float, 3, 1> tt = V * v;

  mat << rotation.r, tt, 0, 0, 0, 1;
  r = rotation.r;
  t = tt;
}

Eigen::Matrix<float, 4, 4> Se3::log() const noexcept {
  So3 rotation(this->r);
  double o;
  Eigen::Matrix<float, 3, 3> w_hat = rotation.log(&o);
  Eigen::Matrix<float, 3, 3> Vinv = Eigen::Matrix<float, 3, 3>::Identity();
  if (o > 1e-12) {
    double c1 = std::sin(o);
    double c2 = (1 - std::cos(o)) / o;
    double k1 = 1 / o / o * (1 - 0.5 * c1 / c2);
    Vinv += -0.5 * w_hat + k1 * w_hat * w_hat;
  }
  Eigen::Matrix<float, 3, 1> v = Vinv * mat.topRightCorner<3, 1>();

  Eigen::Matrix<float, 4, 4> xi_hat = Eigen::Matrix<float, 4, 4>::Zero();
  xi_hat << w_hat, v, 0, 0, 0, 0;
  return xi_hat;
}

Eigen::Matrix<float, 6, 1> Se3::log_vee() const noexcept {
  Eigen::Matrix<float, 4, 4> xi_hat = this->log();
  return vee6(xi_hat);
}

Eigen::Matrix<float, 3, 1> Se3::transform(const Eigen::Matrix<float, 3, 1>& p) const noexcept {
  return r * p + t;
}

Se3 Se3::inverse() const noexcept {
  Eigen::Matrix<float, 4, 4> inv;
  Eigen::Matrix<float, 3, 3> R = r;
  R.transposeInPlace();
  inv << R, -R * this->t, 0, 0, 0, 1;
  return Se3(inv);
}

Eigen::Matrix<float, 6, 6> Se3::adjoint() const noexcept {
  Eigen::Matrix<float, 6, 6> res(Eigen::Matrix<float, 6, 6>::Zero());
  Eigen::Matrix<float, 3, 3> tx = So3::hat(this->t);
  res.topLeftCorner<3, 3>() << r;
  res.bottomRightCorner<3, 3>() << r;
  res.bottomLeftCorner<3, 3>() << tx * r;
  return res;
}

Eigen::Quaternion<float> Se3::quaternion() const noexcept {
  Eigen::Quaternion<float> res = Eigen::Quaternion<float>(r);
  return res.normalized();
}

double Se3::distance(const Se3& rhs) const noexcept {
  return (*this * rhs.inverse()).log_vee().norm();
}

Eigen::Matrix<float, 4, 4> hat6(const Eigen::Matrix<float, 6, 1>& xi) {
  Eigen::Matrix<float, 4, 4> xi_hat;
  xi_hat << 0.0, -xi(2), xi(1), xi(3), xi(2), 0.0, -xi(0), xi(4), -xi(1), xi(0), 0.0, xi(5), 0.0, 0.0, 0.0, 0.0;
  return xi_hat;
}

Eigen::Matrix<float, 6, 1> vee6(const Eigen::Matrix<float, 4, 4>& xi_hat) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << -xi_hat(1, 2), xi_hat(0, 2), -xi_hat(0, 1), xi_hat(0, 3), xi_hat(1, 3), xi_hat(2, 3);
  return xi;
}

}  // namespace superlio
}  // namespace asuka
