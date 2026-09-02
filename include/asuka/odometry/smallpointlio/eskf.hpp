#pragma once

#include <functional>
#include <cstring>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <asuka/odometry/smallpointlio/so3.hpp>

namespace asuka {

namespace smallpointlio {

struct EskfState {
  using ValueType = double;

  constexpr static int DIM = 30;
  constexpr static int position_index = 0;
  constexpr static int rotation_index = 3;
  constexpr static int offset_r_l_i_index = 6;
  constexpr static int offset_t_l_i_index = 9;
  constexpr static int velocity_index = 12;
  constexpr static int omg_index = 15;
  constexpr static int acceleration_index = 18;
  constexpr static int gravity_index = 21;
  constexpr static int bg_index = 24;
  constexpr static int ba_index = 27;

  Eigen::Matrix<ValueType, 3, 1> position = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 3> rotation = Eigen::Matrix<ValueType, 3, 3>::Identity();
  Eigen::Matrix<ValueType, 3, 3> offset_r_l_i = Eigen::Matrix<ValueType, 3, 3>::Identity();
  Eigen::Matrix<ValueType, 3, 1> offset_t_l_i = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> velocity = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> omg = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> acceleration = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> gravity = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> bg = Eigen::Matrix<ValueType, 3, 1>::Zero();
  Eigen::Matrix<ValueType, 3, 1> ba = Eigen::Matrix<ValueType, 3, 1>::Zero();

  EskfState() = default;

  inline void plus(const Eigen::Matrix<ValueType, DIM, 1>& vec) {
    position += vec.segment<3>(position_index);
    rotation *= smallpointlio::exp<ValueType>(vec.segment<3>(rotation_index));
    offset_r_l_i *= smallpointlio::exp<ValueType>(vec.segment<3>(offset_r_l_i_index));
    offset_t_l_i += vec.segment<3>(offset_t_l_i_index);
    velocity += vec.segment<3>(velocity_index);
    omg += vec.segment<3>(omg_index);
    acceleration += vec.segment<3>(acceleration_index);
    gravity += vec.segment<3>(gravity_index);
    bg += vec.segment<3>(bg_index);
    ba += vec.segment<3>(ba_index);
  }
};

struct PointMeasurementResult {
  bool valid;
  EskfState::ValueType z;
  Eigen::Matrix<EskfState::ValueType, 1, 12> h;
  EskfState::ValueType laser_point_cov;
};

struct ImuMeasurementResult {
  bool satu_check[6];
  Eigen::Matrix<EskfState::ValueType, 6, 1> z;
  EskfState::ValueType imu_meas_omg_cov;
  EskfState::ValueType imu_meas_acc_cov;
};

class Eskf {
public:
  using MeasurementModelPoint = std::function<void(const EskfState&, PointMeasurementResult&)>;
  using MeasurementModelImu = std::function<void(const EskfState&, ImuMeasurementResult&)>;

  EskfState x;
  Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> cov;

  Eskf() = default;

  inline void init(const MeasurementModelPoint& h_point, const MeasurementModelImu& h_imu) {
    this->h_point = h_point;
    this->h_imu = h_imu;
  }

  inline void init_timestamp(double timestamp) {
    time_predict_state_last = timestamp;
    time_predict_cov_last = timestamp;
  }

  inline void predict_state(double timestamp) {
    auto dt_state = static_cast<EskfState::ValueType>(timestamp - time_predict_state_last);
    if (dt_state > 0) [[likely]] {
      time_predict_state_last = timestamp;
      x.position += x.velocity * dt_state;
      x.rotation *= smallpointlio::exp<EskfState::ValueType>(x.omg * dt_state);
      x.velocity += (x.rotation * x.acceleration + x.gravity) * dt_state;
    }
  }

  inline void predict_cov(double timestamp, Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM>& noise_cov) {
    auto dt_cov = static_cast<EskfState::ValueType>(timestamp - time_predict_cov_last);
    if (dt_cov > 0) [[likely]] {
      time_predict_cov_last = timestamp;
      Eigen::Matrix<EskfState::ValueType, 3, 1> seg_so3 = -x.omg * dt_cov;
      Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM> f = Eigen::Matrix<EskfState::ValueType, EskfState::DIM, EskfState::DIM>::Identity();
      f.block<3, 3>(EskfState::position_index, EskfState::velocity_index).diagonal().fill(dt_cov);
      f.block<3, 3>(EskfState::rotation_index, EskfState::rotation_index) = smallpointlio::exp<EskfState::ValueType>(seg_so3);
      f.block<3, 3>(EskfState::rotation_index, EskfState::omg_index) = smallpointlio::a_matrix<EskfState::ValueType>(seg_so3) * dt_cov;
      f.block<3, 3>(EskfState::velocity_index, EskfState::rotation_index) = -x.rotation * smallpointlio::hat<EskfState::ValueType>(x.acceleration);
      f.block<3, 3>(EskfState::velocity_index, EskfState::acceleration_index) = x.rotation * dt_cov;
      f.block<3, 3>(EskfState::velocity_index, EskfState::gravity_index).diagonal().fill(dt_cov);
      cov = f * cov * f.transpose() + noise_cov * (dt_cov * dt_cov);
    }
  }

  inline bool update_point() {
    PointMeasurementResult measurement_result;
    h_point(x, measurement_result);
    if (!measurement_result.valid) {
      return false;
    }
    Eigen::Matrix<EskfState::ValueType, EskfState::DIM, 1> pht = cov.template block<EskfState::DIM, 12>(0, 0) * measurement_result.h.transpose();
    EskfState::ValueType temp = measurement_result.h * pht.topRows(12) + measurement_result.laser_point_cov;
    if (temp == 0) [[unlikely]] {
      temp = 1e-6;
    }
    Eigen::Matrix<EskfState::ValueType, EskfState::DIM, 1> k = pht / temp;
    x.plus(k * measurement_result.z);
    cov = cov - k * measurement_result.h * cov.template block<12, EskfState::DIM>(0, 0);
    return true;
  }

  inline bool update_imu() {
    ImuMeasurementResult measurement_result;
    h_imu(x, measurement_result);
    Eigen::Matrix<EskfState::ValueType, 6, 1> z = measurement_result.z;
    Eigen::Matrix<EskfState::ValueType, EskfState::DIM, 6> pht = Eigen::Matrix<EskfState::ValueType, EskfState::DIM, 6>::Zero();
    Eigen::Matrix<EskfState::ValueType, 6, EskfState::DIM> hp = Eigen::Matrix<EskfState::ValueType, 6, EskfState::DIM>::Zero();
    Eigen::Matrix<EskfState::ValueType, 6, 6> Hpht = Eigen::Matrix<EskfState::ValueType, 6, 6>::Zero();
    for (int i = 0; i < 3; i++) {
      if (!measurement_result.satu_check[i]) {
        pht.col(i) = cov.col(EskfState::omg_index + i) + cov.col(EskfState::bg_index + i);
        hp.row(i) = cov.row(EskfState::omg_index + i) + cov.row(EskfState::bg_index + i);
      }
      // NOTE: mirrors the original small_point_lio implementation, which gates the
      // acceleration rows with the GYRO saturation flag satu_check[i] (copy-paste of
      // the block above) instead of satu_check[i + 3].
      if (!measurement_result.satu_check[i]) {
        pht.col(i + 3) = cov.col(EskfState::acceleration_index + i) + cov.col(EskfState::ba_index + i);
        hp.row(i + 3) = cov.row(EskfState::acceleration_index + i) + cov.row(EskfState::ba_index + i);
      }
    }
    for (int i = 0; i < 3; i++) {
      if (!measurement_result.satu_check[i]) {
        Hpht.col(i) = hp.col(EskfState::omg_index + i) + hp.col(EskfState::bg_index + i);
      }
      // NOTE: same as above, mirrors the original gating with the gyro flag.
      if (!measurement_result.satu_check[i]) {
        Hpht.col(i + 3) = hp.col(EskfState::acceleration_index + i) + hp.col(EskfState::ba_index + i);
      }
      Hpht(i, i) += measurement_result.imu_meas_omg_cov;
      Hpht(i + 3, i + 3) += measurement_result.imu_meas_acc_cov;
    }
    Eigen::LDLT<Eigen::Matrix<EskfState::ValueType, 6, 6>> ldlt(Hpht);
    if (ldlt.info() != Eigen::Success) [[unlikely]] {
      return false;
    }
    Eigen::Matrix<EskfState::ValueType, EskfState::DIM, 6> k = pht * ldlt.solve(Eigen::Matrix<EskfState::ValueType, 6, 6>::Identity());
    x.plus(k * z);
    cov -= k * hp;
    return true;
  }

private:
  double time_predict_state_last = 0.0;
  double time_predict_cov_last = 0.0;
  MeasurementModelPoint h_point;
  MeasurementModelImu h_imu;
};

}  // namespace smallpointlio

}  // namespace asuka
