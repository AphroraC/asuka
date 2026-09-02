#include <asuka/modules/imu_prediction.hpp>

namespace asuka {

ImuPrediction::ImuPrediction() {
  const Config config(GlobalConfig::get_config_path("config_extensions"));

  logger = create_module_logger("imu");

  gravity = config.param<double>("imu_prediction", "gravity", 9.81);
  imu_scale = config.param<double>("imu_prediction", "imu_scale", 1.0);
  gravity_vec = Eigen::Vector3d(0.0, 0.0, -gravity);

  on_insert_imu_id =
    Callbacks::on_insert_imu.add(std::bind(&ImuPrediction::on_insert_imu, this, std::placeholders::_1));
  on_new_frame_id = Callbacks::on_new_frame.add(std::bind(&ImuPrediction::on_new_frame, this, std::placeholders::_1));

  kill_switch = false;
  processing_thread_obj = std::thread(&ImuPrediction::processing_thread, this);

  logger->debug("imu prediction (superlio forward integrator): gravity={} imu_scale={}", gravity, imu_scale);
}

ImuPrediction::~ImuPrediction() {
  stop();
  Callbacks::on_insert_imu.remove(on_insert_imu_id);
  Callbacks::on_new_frame.remove(on_new_frame_id);
}

void ImuPrediction::stop() {
  if (!kill_switch.exchange(true)) {
    queue_cv.notify_all();
  }
  if (processing_thread_obj.joinable()) processing_thread_obj.join();
}

void ImuPrediction::on_insert_imu(const ImuData::ConstPtr& imu) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  if (queue.full()) {
    if (++dropped_items % 100 == 1) {
      logger->warn("imu_prediction queue overflow, dropping oldest items ({} dropped)", dropped_items);
    }
  }
  queue.push_back(QueueItem{imu, nullptr});
  queue_cv.notify_one();
}

void ImuPrediction::on_new_frame(const KeyFrame::ConstPtr& frame) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  if (queue.full()) {
    if (++dropped_items % 100 == 1) {
      logger->warn("imu_prediction queue overflow, dropping oldest items ({} dropped)", dropped_items);
    }
  }
  queue.push_back(QueueItem{nullptr, frame});
  queue_cv.notify_one();
}

void ImuPrediction::processing_thread() {
  while (true) {
    QueueItem item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !queue.empty() || kill_switch; });
      if (kill_switch && queue.empty()) return;
      item = queue.front();
      queue.pop_front();
    }

    if (item.imu) {
      process_imu(item.imu);
    } else {
      process_frame(item.frame);
    }
  }
}

void ImuPrediction::process_imu(const ImuData::ConstPtr& imu) {
  if (!initialized) return;

  // Super-LIO ESKF.cpp:136-184 forward Predict, first-sample branch.
  if (forward_time < 0) {
    forward_time = imu->stamp;
    last_imu = *imu;
    return;
  }

  if (!try_integrate(imu) && imu->stamp > forward_time) {
    if (pending_imus.full()) {
      if (++dropped_items % 100 == 1) {
        logger->warn("imu_prediction pending buffer overflow, dropping oldest ({} dropped)", dropped_items);
      }
    }
    pending_imus.push_back(imu);
  }
}

bool ImuPrediction::try_integrate(const ImuData::ConstPtr& imu) {
  const double dt = imu->stamp - forward_time;
  if (dt < 0 || dt > 0.2) return false;

  Eigen::Vector3d acc = 0.5 * (imu->linear_acc + last_imu.linear_acc);
  acc = imu_scale * acc;
  acc = acc - ba;

  const Eigen::Vector3d gyr = 0.5 * (imu->angular_vel + last_imu.angular_vel) - bg;

  const Eigen::Vector3d new_p = fw_pos + fw_vel * dt + 0.5 * (fw_rot * acc) * dt * dt + 0.5 * gravity_vec * dt * dt;
  const Eigen::Vector3d new_v = fw_vel + fw_rot * acc * dt + gravity_vec * dt;
  const Eigen::Matrix3d new_r = fw_rot * exp_so3(gyr, dt);

  fw_rot = new_r;
  fw_vel = new_v;
  fw_pos = new_p;

  forward_time = imu->stamp;
  last_imu = *imu;

  emit_predicted_odometry(imu->stamp, fw_rot, fw_pos, fw_vel);
  return true;
}

void ImuPrediction::process_frame(const KeyFrame::ConstPtr& frame) {
  // Integrate held samples that belong to the window before this anchor,
  // then re-anchor, then continue with the rest: this reproduces the
  // original's real-time order (IMUs predicted between scans, Update()
  // re-anchoring the forward state afterwards).
  for (const auto& imu : pending_imus) {
    if (imu->stamp < frame->stamp) try_integrate(imu);
  }

  // Re-anchor the forward integrator at the frontend keyframe, mirroring
  // Super-LIO's Update(): fw_R_/fw_p_/fw_v_ and forward_time_ are reset;
  // forward_last_imu_ is deliberately left untouched (original behavior).
  fw_rot = frame->T_world_imu.rotation();
  fw_pos = frame->T_world_imu.translation();
  fw_vel = frame->v_world_imu;
  bg = frame->imu_bias.head<3>();
  ba = frame->imu_bias.tail<3>();
  forward_time = frame->stamp;
  initialized = true;

  std::vector<ImuData::ConstPtr> still_pending;
  for (const auto& imu : pending_imus) {
    if (imu->stamp >= frame->stamp && !try_integrate(imu)) {
      still_pending.push_back(imu);
    }
  }
  pending_imus.clear();
  for (const auto& imu : still_pending) {
    pending_imus.push_back(imu);
  }
}

void ImuPrediction::emit_predicted_odometry(
  double stamp,
  const Eigen::Matrix3d& R,
  const Eigen::Vector3d& p,
  const Eigen::Vector3d& v) {
  // Predicted poses are not keyframes: no id/bias/cloud, matching the
  // frontend KeyFrame semantics loosely enough for trajectory output.
  auto frame = std::make_shared<KeyFrame>();
  frame->id = -1;
  frame->stamp = stamp;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = Eigen::Isometry3d::Identity();
  frame->T_world_imu.linear() = R;
  frame->T_world_imu.translation() = p;
  frame->v_world_imu = v;
  Callbacks::on_odometry_imu(frame);
}

}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::ImuPrediction();
}
