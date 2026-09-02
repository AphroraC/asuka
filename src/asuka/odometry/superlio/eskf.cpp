#include "asuka/odometry/superlio/eskf.hpp"

namespace asuka {
namespace superlio {

static inline M3d right_jacobian_so3(const V3& ang_vel, const Scalar& dt) {
  const V3d w = ang_vel.template cast<double>();
  const double dt_d = static_cast<double>(dt);
  M3d jr = M3d::Identity();
  if (!w.allFinite() || !std::isfinite(dt_d)) {
    return jr;
  }
  const V3d phi = w * dt_d;
  const double theta = phi.norm();
  if (!std::isfinite(theta)) {
    return jr;
  }
  M3d k;
  k << 0.0, -phi.z(), phi.y(), phi.z(), 0.0, -phi.x(), -phi.y(), phi.x(), 0.0;

  const M3d k2 = k * k;
  double a, b;
  if (theta < 1e-6) {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    a = 0.5 - theta2 / 24.0 + theta4 / 720.0;
    b = 1.0 / 6.0 - theta2 / 120.0 + theta4 / 5040.0;
  } else {
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    a = (1.0 - std::cos(theta)) / theta2;
    b = (theta - std::sin(theta)) / theta3;
  }
  jr = M3d::Identity() - a * k + b * k2;
  return jr;
}

void Eskf::set_initial_conditions(
  Options options,
  const V3& init_bg,
  const V3& init_ba,
  const float imu_scale,
  const V3& gravity) {
  build_noise(options);
  this->options = options;
  bg = init_bg;
  ba = init_ba;
  g = gravity;
  this->imu_scale = imu_scale;
  gravity_norm = gravity.norm();

  cov = 1e-4 * M18::Identity();
  cov.template block<3, 3>(0, 0) = 0.1 * M_PI / 180.0 * M3::Identity();
}

void Eskf::set_x(const SysState& x) {
  last_imu_time = x.timestamp;
  current_time = last_imu_time;
  rot = x.rot;
  p = x.p;
  v = x.v;
  bg = x.bg;
  ba = x.ba;
}

void Eskf::build_noise(const Options& options) {
  double et = options.gyro_var;
  double ev = options.acce_var;
  double eg = options.bias_gyro_var;
  double ea = options.bias_acce_var;

  double et2 = et;
  double ev2 = ev;
  double eg2 = eg;
  double ea2 = ea;

  q_noise.diagonal() << et2, et2, et2, ev2, ev2, ev2, eg2, eg2, eg2, ea2, ea2, ea2;
}

void Eskf::update() {
  rot = rot * So3::exp(dx.template block<3, 1>(0, 0));
  p += dx.template block<3, 1>(3, 0);
  v += dx.template block<3, 1>(6, 0);

  bg += dx.template block<3, 1>(9, 0);
  ba += dx.template block<3, 1>(12, 0);

  g += dx.template block<3, 1>(15, 0);
  g = gravity_norm * (g.normalized());
}

bool Eskf::predict(const ImuSample& imu) {
  if (last_imu_time < 0) {
    last_imu_time = imu.secs;
    last_imu = imu;
    return false;
  }

  if (imu.secs <= last_obs_time) {
    last_imu_time = imu.secs;
    last_imu = imu;
    return false;
  }

  current_time = imu.secs;

  double dt;
  if (last_imu_time < last_obs_time) {
    dt = imu.secs - last_obs_time;
  } else if (imu.secs > current_obs_time) {
    dt = current_obs_time - last_imu_time;
    current_time = current_obs_time;
  } else {
    dt = imu.secs - last_imu_time;
  }

  V3 acc = 0.5 * (imu.acc + last_imu.acc);
  acc = imu_scale * acc;
  acc = acc - ba;
  body_omega = 0.5 * (imu.gyr + last_imu.gyr) - bg;
  M3 jr_dt = (dt * right_jacobian_so3(body_omega, dt)).cast<Scalar>();

  M3 r_m3 = rot.r;
  M3 r_dt = r_m3 * dt;

  Fx f_x = Fx::Identity();
  f_x.template block<3, 3>(0, 0) = So3::exp(-body_omega, dt).r;
  f_x.template block<3, 3>(0, 9) = -jr_dt;
  f_x.template block<3, 3>(3, 6) = M3::Identity() * dt;
  f_x.template block<3, 3>(6, 0) = -r_m3 * So3::hat(acc) * dt;
  f_x.template block<3, 3>(6, 12) = -r_dt;
  f_x.template block<3, 3>(6, 15) = M3::Identity() * dt;

  Fw f_w = Fw::Zero();
  f_w.template block<3, 3>(0, 0) = -jr_dt;
  f_w.template block<3, 3>(6, 3) = -r_dt;
  f_w.template block<3, 3>(9, 6) = M3::Identity() * dt;
  f_w.template block<3, 3>(12, 9) = M3::Identity() * dt;

  cov = f_x * cov * f_x.transpose() + f_w * q_noise * f_w.transpose();

  global_acc = rot.r * acc + g;
  p = p + v * dt + 0.5 * global_acc * dt * dt;
  v = v + global_acc * dt;
  rot = rot * So3::exp(body_omega, dt);

  last_imu_time = imu.secs;
  last_imu = imu;
  return true;
}

bool Eskf::update_observe(Eskf::ObsFunc obs) {
  So3 r_pred = rot;
  V3 p_pred = p;
  V3 v_pred = v;
  V3 bg_pred = bg;
  V3 ba_pred = ba;
  V3 g_pred = g;

  M18 cov_pred = cov;

  M6 htvh;
  V6 htvr;

  M18 pk = M18::Zero();
  M18 qk = M18::Zero();
  M18 k_x = M18::Zero();

  need_converge = false;

  for (int iter = 0; iter < options.num_iterations; ++iter) {
    if (iter > 2) {
      need_converge = true;
    }

    obs(get_kf_state(), htvh, htvr);

    V18 dx_prior = V18::Zero();
    dx_prior.template block<3, 1>(0, 0) = (r_pred.inverse() * rot).log_vee();
    dx_prior.template block<3, 1>(3, 0) = p - p_pred;
    dx_prior.template block<3, 1>(6, 0) = v - v_pred;
    dx_prior.template block<3, 1>(9, 0) = bg - bg_pred;
    dx_prior.template block<3, 1>(12, 0) = ba - ba_pred;
    dx_prior.template block<3, 1>(15, 0) = g - g_pred;

    M18 g_prior = M18::Identity();

    M3 j_prior = M3::Identity() - 0.5 * So3::hat(dx_prior.template block<3, 1>(0, 0));

    g_prior.template block<3, 3>(0, 0) = j_prior;

    pk = g_prior * cov_pred * g_prior.transpose();

    dx_prior = g_prior * dx_prior;

    M18 htrh = M18::Zero();
    htrh.template block<6, 6>(0, 0) = htvh;

    M18 a = pk.inverse() + htrh;
    qk = a.inverse();

    V18 b = V18::Zero();
    b.template head<6>() = htvr;

    k_x = qk * htrh;

    dx = qk * b + (k_x - M18::Identity()) * dx_prior;

    update();

    if (dx.lpNorm<Eigen::Infinity>() < options.quit_eps && iter > 0) {
      break;
    }
  }

  cov = qk;

  M18 g_reset = M18::Identity();
  M3 j_reset = M3::Identity() - 0.5 * So3::hat(dx.template block<3, 1>(0, 0));

  g_reset.template block<3, 3>(0, 0) = j_reset;

  cov = g_reset * cov * g_reset.transpose();

  cov = 0.5 * (cov + cov.transpose());

  dx.setZero();

  last_obs_time = current_obs_time;
  return true;
}

}  // namespace superlio
}  // namespace asuka
