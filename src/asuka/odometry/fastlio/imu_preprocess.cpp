#include <asuka/odometry/fastlio/imu_preprocess.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace fastlio {

ImuPreprocess::ImuPreprocess() {
  logger = create_module_logger("preprocess");
  reset();

  const Config sensor_config(GlobalConfig::get_config_path("config_sensor"));
  std::string lidar_key;
  if (sensor_config.has("livox"))
    lidar_key = "livox";
  else if (sensor_config.has("robosense"))
    lidar_key = "robosense";
  else
    throw std::runtime_error("config_sensor: no active lidar block");
  std::vector<std::string> nest{lidar_key};
  lidar_T_wrt_imu = sensor_config.param_nested<Eigen::Vector3d>(nest, "extrinsic_T", Eigen::Vector3d::Zero());
  lidar_R_wrt_imu = sensor_config.param_nested<Eigen::Matrix3d>(nest, "extrinsic_R", Eigen::Matrix3d::Identity());
  cov_gyro_scale = sensor_config.param_nested<Eigen::Vector3d>(nest, "gyro_noise", Eigen::Vector3d::Constant(0.1));
  cov_acc_scale = sensor_config.param_nested<Eigen::Vector3d>(nest, "acc_noise", Eigen::Vector3d::Constant(0.1));
  cov_bias_gyro = sensor_config.param_nested<Eigen::Vector3d>(nest, "gyro_bias", Eigen::Vector3d::Constant(0.0001));
  cov_bias_acc = sensor_config.param_nested<Eigen::Vector3d>(nest, "acc_bias", Eigen::Vector3d::Constant(0.0001));

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  gravity_estimation = odometry_config.param<bool>("mapping", "gravity_estimation", false);
}

void ImuPreprocess::reset() {
  imu_poses.clear();
  mean_acc.setZero();
  mean_gyro.setZero();
  angvel_last.setZero();
  acc_s_last.setZero();
  cov_acc.setZero();
  cov_gyro.setZero();
  imu_need_init = true;
  first_lidar_time = 0.0;
  last_lidar_end_time = 0.0;
  init_iter_num = 1;
  b_first_frame = true;
}

void ImuPreprocess::initialize(double lidar_start_time) {
  first_lidar_time = lidar_start_time;
}

void ImuPreprocess::process(
  const ImuMeasureGroup& meas,
  esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state,
  PointCloudT::Ptr& pcl_out) {
  if (meas.imu.empty()) {
    return;
  }

  if (imu_need_init) {
    imu_init(meas, kf_state);
    last_imu = meas.imu.back();
    if (init_iter_num > max_ini_count) {
      cov_acc = cov_acc * std::pow(g_m_s2 / mean_acc.norm(), 2);
      imu_need_init = false;
      cov_acc = cov_acc_scale;
      cov_gyro = cov_gyro_scale;
      logger->debug("IMU initialization done");
    }
    return;
  }

  undistort_pcl(meas, kf_state, *pcl_out);
}

void ImuPreprocess::imu_init(
  const ImuMeasureGroup& meas,
  esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state) {
  if (b_first_frame) {
    reset();
    init_iter_num = 1;
    b_first_frame = false;
    const auto& imu = meas.imu.front();
    mean_acc = imu->linear_acc;
    mean_gyro = imu->angular_vel;
    first_lidar_time = meas.lidar_beg_time;
  }

  int N = init_iter_num;
  for (const auto& imu : meas.imu) {
    const Eigen::Vector3d& cur_acc = imu->linear_acc;
    const Eigen::Vector3d& cur_gyro = imu->angular_vel;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyro += (cur_gyro - mean_gyro) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyro =
      cov_gyro * (N - 1.0) / N + (cur_gyro - mean_gyro).cwiseProduct(cur_gyro - mean_gyro) * (N - 1.0) / (N * N);
    N++;
  }
  init_iter_num = N;

  ikfom::state_ikfom init_state = kf_state.get_x();

  if (gravity_estimation) {
    Eigen::Vector3d mean_acc_normalized = mean_acc.normalized();
    Eigen::Quaterniond q_init = Eigen::Quaterniond::FromTwoVectors(mean_acc_normalized, Eigen::Vector3d(0, 0, 1));
    init_state.rot = q_init.normalized();
    init_state.grav = ikfom::S2(Eigen::Vector3d(0, 0, -g_m_s2));
  } else {
    init_state.grav = ikfom::S2(-mean_acc / mean_acc.norm() * g_m_s2);
  }

  init_state.bg = mean_gyro;
  init_state.offset_T_L_I = lidar_T_wrt_imu;
  init_state.offset_R_L_I = ikfom::SO3(lidar_R_wrt_imu);
  kf_state.change_x(init_state);

  esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  kf_state.change_P(init_P);

  last_imu = meas.imu.back();
}

void ImuPreprocess::undistort_pcl(
  const ImuMeasureGroup& meas,
  esekfom::esekf<ikfom::state_ikfom, 12, ikfom::input_ikfom>& kf_state,
  PointCloudT& pcl_out) {
  if (meas.imu.empty()) return;

  const double imu_beg_time = meas.imu.front()->stamp;
  const double imu_end_time = meas.imu.back()->stamp;
  const double pcl_beg_time = meas.lidar_beg_time;
  const double pcl_end_time = meas.lidar_end_time;

  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), [](const PointT& a, const PointT& b) {
    return a.curvature < b.curvature;
  });

  ikfom::state_ikfom imu_state = kf_state.get_x();
  imu_poses.clear();
  imu_poses.push_back(
    ikfom::set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  Eigen::Matrix3d R_imu;
  double dt = 0.0;

  ikfom::input_ikfom in;
  Eigen::Matrix<double, 12, 12> Q = ikfom::process_noise_cov();

  auto get_imu = [&](int idx) -> const ImuData::ConstPtr& { return (idx < 0) ? last_imu : meas.imu[idx]; };

  const int imu_size = static_cast<int>(meas.imu.size());
  for (int i = -1; i < imu_size - 1; ++i) {
    const auto& head = get_imu(i);
    const auto& tail = get_imu(i + 1);

    if (tail->stamp < last_lidar_end_time) continue;

    angvel_avr = 0.5 * (head->angular_vel + tail->angular_vel);

    if (head->angular_vel.maxCoeff() > 25.0) {
      angvel_avr = tail->angular_vel;
      logger->warn("bad imu");
    }
    if (tail->angular_vel.maxCoeff() > 25.0) {
      angvel_avr = head->angular_vel;
      logger->warn("bad imu");
    }
    if (head->angular_vel.maxCoeff() > 25.0 || tail->angular_vel.maxCoeff() > 25.0) {
      logger->warn("bad imu, skipping");
      continue;
    }

    acc_avr = 0.5 * (head->linear_acc + tail->linear_acc);
    acc_avr = acc_avr * g_m_s2 / mean_acc.norm();

    if (head->stamp < last_lidar_end_time) {
      dt = tail->stamp - last_lidar_end_time;
    } else {
      dt = tail->stamp - head->stamp;
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyro;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyro;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
    for (int j = 0; j < 3; ++j) {
      acc_s_last[j] += imu_state.grav[j];
    }
    double offs_t = tail->stamp - pcl_beg_time;
    imu_poses.push_back(
      ikfom::set_pose6d(
        offs_t,
        acc_s_last,
        angvel_last,
        imu_state.vel,
        imu_state.pos,
        imu_state.rot.toRotationMatrix()));
  }

  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  last_imu = meas.imu.back();
  last_lidar_end_time = pcl_end_time;

  if (pcl_out.points.empty()) return;

  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = imu_poses.end() - 1; it_kp != imu_poses.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu = head->rot;
    vel_imu = head->vel;
    pos_imu = head->pos;
    acc_imu = tail->acc;
    angvel_avr = tail->gyro;

    for (; it_pcl->curvature / 1000.0 > head->offset_time; it_pcl--) {
      dt = it_pcl->curvature / 1000.0 - head->offset_time;

      Eigen::Matrix3d R_i(R_imu * ikfom::expSo3(Eigen::Vector3d(angvel_avr * dt)));

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      Eigen::Vector3d P_compensate =
        imu_state.offset_R_L_I.conjugate() *
        (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
         imu_state.offset_T_L_I);

      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

}  // namespace fastlio
}  // namespace asuka
