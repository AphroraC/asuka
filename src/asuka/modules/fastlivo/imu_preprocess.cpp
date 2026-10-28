#include <asuka/modules/fastlivo/imu_preprocess.hpp>

#include <omp.h>

#include <spdlog/spdlog.h>

#include <asuka/utility/logging.hpp>

namespace asuka {
namespace fastlivo {

ImuPreprocess::ImuPreprocess()
: Eye3d(Eigen::Matrix3d::Identity()),
  Zero3d(0, 0, 0),
  b_first_frame(true),
  imu_need_init(true) {
  logger = create_module_logger("odometry");
  init_iter_num = 1;
  cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_bias_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_bias_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_inv_expo = 0.2;
  mean_acc = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr = Eigen::Vector3d(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu = std::make_shared<ImuData>();
  last_prop_end_time = 0.0;
  time_last_scan = 0.0;
  IMU_mean_acc_norm = 0.0;
  first_lidar_time = 0.0;
  unbiased_gyr = Zero3d;
}

ImuPreprocess::~ImuPreprocess() {}

void ImuPreprocess::Reset() {
  logger->warn("Reset ImuPreprocess");
  mean_acc = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr = Eigen::Vector3d(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init = true;
  init_iter_num = 1;
  IMUpose.clear();
  last_imu = std::make_shared<ImuData>();
}

void ImuPreprocess::disable_gravity_est() {
  gravity_est_en = false;
}

void ImuPreprocess::disable_bias_est() {
  ba_bg_est_en = false;
}

void ImuPreprocess::disable_exposure_est() {
  exposure_estimate_en = false;
}

void ImuPreprocess::set_extrinsic(const Eigen::Vector3d& transl, const Eigen::Matrix3d& rot) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuPreprocess::set_gyr_cov_scale(const Eigen::Vector3d& scaler) {
  cov_gyr = scaler;
}

void ImuPreprocess::set_acc_cov_scale(const Eigen::Vector3d& scaler) {
  cov_acc = scaler;
}

void ImuPreprocess::set_gyr_bias_cov(const Eigen::Vector3d& b_g) {
  cov_bias_gyr = b_g;
}

void ImuPreprocess::set_inv_expo_cov(const double& inv_expo) {
  cov_inv_expo = inv_expo;
}

void ImuPreprocess::set_acc_bias_cov(const Eigen::Vector3d& b_a) {
  cov_bias_acc = b_a;
}

void ImuPreprocess::set_imu_init_frame_num(const int& num) {
  MAX_INI_COUNT = num;
}

void ImuPreprocess::IMU_init(const MeasureGroup& meas, StatesGroup& state_inout, int& N) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  logger->info("IMU Initializing: {:.1f} %", static_cast<double>(N) / MAX_INI_COUNT * 100);
  Eigen::Vector3d cur_acc, cur_gyr;

  if (b_first_frame) {
    Reset();
    N = 1;
    b_first_frame = false;
    const auto& imu_acc = meas.imu.front()->linear_acc;
    const auto& gyr_acc = meas.imu.front()->angular_vel;
    mean_acc = imu_acc;
    mean_gyr = gyr_acc;
  }

  for (const auto& imu : meas.imu) {
    cur_acc = imu->linear_acc;
    cur_gyr = imu->angular_vel;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }
  IMU_mean_acc_norm = mean_acc.norm();
  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
  state_inout.rot_end = Eye3d;
  state_inout.bias_g = Zero3d;

  last_imu = meas.imu.back();
}

void ImuPreprocess::UndistortPcl(LidarMeasureGroup& lidar_meas, StatesGroup& state_inout, PointCloudT& pcl_out) {
  double t0 = omp_get_wtime();
  (void)t0;
  pcl_out.clear();
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  MeasureGroup& meas = lidar_meas.measures.back();
  auto v_imu = meas.imu;
  v_imu.insert(v_imu.begin(), last_imu);
  const double imu_beg_time = v_imu.front()->stamp;
  const double imu_end_time = v_imu.back()->stamp;
  const double prop_beg_time = last_prop_end_time;

  const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

  if (lidar_meas.lio_vio_flg == LIO) {
    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;
    IMUpose.push_back(
      set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
  }

  /*** std::forward propagation at each imu point ***/
  Eigen::Vector3d acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end),
    pos_imu(state_inout.pos_end);
  Eigen::Matrix3d R_imu(state_inout.rot_end);
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> F_x, cov_w;
  double dt, dt_all = 0.0;
  double offs_t;
  double tau;
  if (!imu_time_init) {
    tau = 1.0;
    imu_time_init = true;
  } else {
    tau = state_inout.inv_expo_time;
  }
  switch (lidar_meas.lio_vio_flg) {
    case LIO:
    case VIO:
      dt = 0;
      for (int i = 0; i < static_cast<int>(v_imu.size()) - 1; i++) {
        auto head = v_imu[i];
        auto tail = v_imu[i + 1];

        if (tail->stamp < prop_beg_time) continue;

        angvel_avr = 0.5 * (head->angular_vel + tail->angular_vel);
        acc_avr = 0.5 * (head->linear_acc + tail->linear_acc);

        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

        if (head->stamp < prop_beg_time) {
          dt = tail->stamp - last_prop_end_time;
          offs_t = tail->stamp - prop_beg_time;
        } else if (i != static_cast<int>(v_imu.size()) - 2) {
          dt = tail->stamp - head->stamp;
          offs_t = tail->stamp - prop_beg_time;
        } else {
          dt = prop_end_time - head->stamp;
          offs_t = prop_end_time - prop_beg_time;
        }

        dt_all += dt;

        /* covariance propagation */
        Eigen::Matrix3d acc_avr_skew;
        Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

        F_x.setIdentity();
        cov_w.setZero();

        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
        if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
        F_x.block<3, 3>(3, 7) = Eye3d * dt;
        F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
        if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
        if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

        if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
        cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
        cov_w.block<3, 3>(7, 7) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt;  // bias gyro covariance
        cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt;  // bias acc covariance

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

        /* propogation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* propogation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;

        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
      }

      lidar_meas.last_lio_update_time = prop_end_time;
      break;
    default:
      break;
  }

  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  last_imu = v_imu.back();
  last_prop_end_time = prop_end_time;

  double t1 = omp_get_wtime();
  (void)t1;
  (void)imu_beg_time;
  (void)imu_end_time;
  (void)dt_all;

  if (pcl_wait_proc.points.size() < 1) return;

  /*** undistort each lidar point (backward propagation), ONLY working for LIO
   * update ***/
  if (lidar_meas.lio_vio_flg == LIO) {
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    Eigen::Matrix3d extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    Eigen::Vector3d exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);
      (void)tail;

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
        Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate =
          (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
}

void ImuPreprocess::Process2(LidarMeasureGroup& lidar_meas, StatesGroup& stat, PointCloudT::Ptr cur_pcl_un_) {
  assert(lidar_meas.lidar != nullptr);

  MeasureGroup meas = lidar_meas.measures.back();

  if (imu_need_init) {
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
    (void)pcl_end_time;

    if (meas.imu.empty()) {
      return;
    }
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init = true;

    last_imu = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT) {
      imu_need_init = false;
      logger->info(
        "IMU Initials: Gravity: {:.4f} {:.4f} {:.4f} {:.4f}; acc covarience: {:.8f} {:.8f} {:.8f}; gry covarience: "
        "{:.8f} {:.8f} {:.8f}",
        stat.gravity[0],
        stat.gravity[1],
        stat.gravity[2],
        mean_acc.norm(),
        cov_acc[0],
        cov_acc[1],
        cov_acc[2],
        cov_gyr[0],
        cov_gyr[1],
        cov_gyr[2]);
      logger->info(
        "IMU Initials: ba covarience: {:.8f} {:.8f} {:.8f}; bg covarience: {:.8f} {:.8f} {:.8f}",
        cov_bias_acc[0],
        cov_bias_acc[1],
        cov_bias_acc[2],
        cov_bias_gyr[0],
        cov_bias_gyr[1],
        cov_bias_gyr[2]);
    }

    return;
  }

  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
}

}  // namespace fastlivo
}  // namespace asuka
