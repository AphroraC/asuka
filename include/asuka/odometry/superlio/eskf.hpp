#pragma once

#include <functional>
#include <memory>

#include <asuka/odometry/superlio/alias.hpp>
#include <asuka/odometry/superlio/manifold.hpp>
#include <asuka/odometry/superlio/types.hpp>

namespace asuka {
namespace superlio {

class Eskf {
public:
  using Ptr = std::shared_ptr<Eskf>;
  using State = Eigen::Matrix<Scalar, 18, 1>;      // Flatten: R p v bg ba g
  using StateDof = Eigen::Matrix<Scalar, 17, 1>;   // Flatten: R p v bg ba g_2
  using Noise = Eigen::Matrix<Scalar, 12, 12>;
  using Cov = Eigen::Matrix<Scalar, 18, 18>;
  using Fx = Eigen::Matrix<Scalar, 18, 18>;
  using Fw = Eigen::Matrix<Scalar, 18, 12>;

  struct Options {
    Options() {}
    int num_iterations = 3;
    double quit_eps = 1e-6;

    double gyro_var = 1e-5;
    double acce_var = 1e-2;
    double bias_gyro_var = 1e-6;
    double bias_acce_var = 1e-4;
  };

  struct KfState {
    bool need_converge = true;
    Se3 pose;
  };

  Eskf(Options option = Options()) : options(option) { build_noise(option); }

  Eskf(Options options, const V3& init_bg, const V3& init_ba,
       const V3& gravity = V3(0, 0, -9.8f))
      : options(options) {
    build_noise(options);
    bg = init_bg;
    ba = init_ba;
    g = gravity;
  }

  void set_initial_conditions(Options options, const V3& init_bg, const V3& init_ba,
                              const float imu_scale = 1.0f, const V3& gravity = V3(0, 0, -9.8f));

  bool predict(const ImuSample& imu);

  using ObsFunc = std::function<void(const KfState& kf_state, M6& ht_vinv_h, V6& ht_vinv_r)>;
  bool update_observe(ObsFunc obs);

  double get_time() const { return current_time; }

  SysState get_sys_state() const { return SysState(current_time, rot, p, v, bg, ba); }

  NavState get_nav_state() const { return NavState(current_time, rot, p, v); }

  DynamicState get_dynamic_state() const {
    return DynamicState(current_time, rot.r, p, v, body_omega, global_acc);
  }

  KfState get_kf_state() const { return KfState{need_converge, get_se3()}; }

  PoseT get_pose_t() const { return PoseT(current_time, rot, p); }

  Cov get_cov() const { return cov; }

  Se3 get_se3() const { return Se3(rot, p); }

  void set_obs_time(double obs_time) { current_obs_time = obs_time; }
  void set_last_obs_time(double obs_time) { last_obs_time = obs_time; }

  void set_x(const SysState& x);

  void set_cov(const Cov& c) { cov = c; }

  V3 get_gravity() const { return g; }

  bool inited = false;

private:
  void build_noise(const Options& options);
  void update();

  bool need_converge = true;
  float imu_scale = 1.0f;
  ImuSample last_imu;
  double current_time = 0.0;
  double last_imu_time = -1.0;
  double last_obs_time = 0.0;
  double current_obs_time = 0.0;
  double gravity_norm = 9.81;

  So3 rot;
  V3 p = V3::Zero();
  V3 v = V3::Zero();
  V3 bg = V3::Zero();
  V3 ba = V3::Zero();
  V3 g{0, 0, -9.8f};
  V3 global_acc = V3::Zero();
  V3 body_omega = V3::Zero();

  State dx = State::Zero();

  Cov cov = Cov::Identity();

  Noise q_noise = Noise::Zero();

  Options options;
};

}  // namespace superlio
}  // namespace asuka
