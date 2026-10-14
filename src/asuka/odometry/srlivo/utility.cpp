#include <asuka/odometry/srlivo/utility.hpp>

#include <algorithm>

namespace asuka {
namespace srlivo {

bool time_list(point3D &point_1, point3D &point_2) {
  return (point_1.relative_time < point_2.relative_time);
}

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb) {
  double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
  norm = std::acos(norm) * 180 / M_PI;
  return norm;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b) {
  Eigen::Matrix3d rota = q_a.toRotationMatrix();
  Eigen::Matrix3d rotb = q_b.toRotationMatrix();

  double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
  norm = std::acos(norm) * 180 / M_PI;
  return norm;
}

double AngularDistance(const Eigen::Vector3d &d_so3) {
  Eigen::Matrix3d d_R = numType::so3ToRotation(d_so3);

  double norm = (d_R.trace() - 1) / 2;
  norm = std::acos(norm) * 180 / M_PI;
  return norm;
}

void subSampleFrame(std::vector<point3D> &frame, double size_voxel) {
  std::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
  for (int i = 0; i < (int)frame.size(); i++) {
    auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
    auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
    auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
    grid[voxel(kx, ky, kz)].push_back(frame[i]);
  }
  frame.resize(0);
  int step = 0;
  for (const auto &n : grid) {
    if (n.second.size() > 0) {
      frame.push_back(n.second[0]);
      step++;
    }
  }
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling) {
  keypoints.resize(0);
  std::vector<point3D> frame_sub;
  frame_sub.resize(frame.size());
  for (int i = 0; i < (int)frame_sub.size(); i++) {
    frame_sub[i] = frame[i];
  }
  subSampleFrame(frame_sub, size_voxel_subsampling);
  keypoints.reserve(frame_sub.size());
  for (int i = 0; i < (int)frame_sub.size(); i++) {
    keypoints.push_back(frame_sub[i]);
  }
}

void distortFrameByConstant(std::vector<point3D> &points, std::vector<imuState> &imu_states, double time_frame_begin, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
  double time_frame_end, time_point;

  time_frame_end = imu_states.back().timestamp;

  Eigen::Quaterniond quat_begin = imu_states.front().quat;
  Eigen::Quaterniond quat_end = imu_states.back().quat;

  Eigen::Vector3d trans_begin = imu_states.front().trans;
  Eigen::Vector3d trans_end = imu_states.back().trans;

  std::vector<point3D>::iterator iter = points.begin();

  while (iter != points.end()) {
    time_point = time_frame_begin + (*iter).relative_time / 1000.0;

    if (fabs(time_point - time_frame_begin) < 1e-6) time_point = time_frame_begin + 1e-6;
    if (fabs(time_point - time_frame_end) < 1e-6) time_point = time_frame_end - 1e-6;

    double alpha_time = (time_point - time_frame_begin) / (time_frame_end - time_frame_begin);

    if (alpha_time > 1) alpha_time = 1;
    if (alpha_time < 0) alpha_time = 0;

    Eigen::Quaterniond quat_alpha = quat_begin.slerp(alpha_time, quat_end);
    Eigen::Vector3d trans_alpha = (1.0 - alpha_time) * trans_begin + alpha_time * trans_end;

    (*iter).imu_point = quat_alpha.toRotationMatrix() * (R_imu_lidar * (*iter).raw_point + t_imu_lidar) + trans_alpha;

    iter++;
  }
}

void distortFrameByImu(std::vector<point3D> &points, std::vector<imuState> &imu_states, double time_frame_begin, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
  double time_imu_begin, time_imu_end, time_point;

  Eigen::Quaterniond quat_imu;
  Eigen::Vector3d trans_imu, vel_imu;
  Eigen::Vector3d un_acc, un_gyr;

  std::vector<point3D>::iterator iter = points.begin();

  for (int n = 0; n + 1 < imu_states.size(); n++) {
    // distortion method 1
    time_imu_begin = imu_states[n].timestamp;
    quat_imu = imu_states[n].quat;
    trans_imu = imu_states[n].trans;
    vel_imu = imu_states[n].vel;

    time_imu_end = imu_states[n + 1].timestamp;
    un_acc = imu_states[n + 1].un_acc;
    un_gyr = imu_states[n + 1].un_gyr;
    // distortion method 1

    while (iter != points.end()) {
      time_point = time_frame_begin + (*iter).relative_time / 1000.0;

      if (time_point > time_imu_begin - 1e-6 && time_point < time_imu_end + 1e-6) {
        if (fabs(time_point - time_imu_begin) < 1e-6) time_point = time_imu_begin + 1e-6;
        if (fabs(time_point - time_imu_end) < 1e-6) time_point = time_imu_end - 1e-6;

        // distortion method 1
        double dt = time_point - time_imu_begin;

        Eigen::Quaterniond quat_point = quat_imu * numType::so3ToQuat(un_gyr * dt);
        quat_point.normalize();
        Eigen::Vector3d trans_point = trans_imu + vel_imu * dt + 0.5 * un_acc * dt * dt;

        (*iter).imu_point = quat_point.toRotationMatrix() * (R_imu_lidar * (*iter).raw_point + t_imu_lidar) + trans_point;
        // distortion method 1

        iter++;
      } else {
        break;
      }
    }
  }
}

void transformPoint(point3D &point_temp, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
  point_temp.point = q_end.toRotationMatrix() * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t_end;
}

void transformAllImuPoint(std::vector<point3D> &points, std::vector<imuState> &imu_states, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
  Eigen::Quaterniond quat_end_inv = imu_states.back().quat.inverse();
  Eigen::Vector3d trans_end_inv = -quat_end_inv.toRotationMatrix() * imu_states.back().trans;

  std::vector<point3D>::iterator iter = points.begin();

  while (iter != points.end()) {
    (*iter).raw_point = R_imu_lidar.transpose() * (quat_end_inv.toRotationMatrix() * (*iter).imu_point + trans_end_inv) - R_imu_lidar.transpose() * t_imu_lidar;
    iter++;
  }
}

}  // namespace srlivo
}  // namespace asuka
