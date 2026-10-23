#include <asuka/modules/zlio/odometry_estimation.hpp>

#include <algorithm>

#include <asuka/core/callbacks.hpp>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace zlio {

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  config = loadConfig(load_config("config_odometry"));
  const asuka::Config plugin_config = load_config("config_odometry");
  param.output_resolution = plugin_config.param<double>("odometry", "output_resolution", 0.1);
  const asuka::Config ros_config = load_config("config_ros");
  param.save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);

  odometry = std::make_unique<Odometry>(config);
  // 上游 FrameResult.cloud 是受 setOutputResolution 门控的独立导出流（默认 0 =
  // 不导出）。asuka 的 KeyFrame 云与存图都消费该流，必须显式开启。
  odometry->setOutputResolution(param.output_resolution);

  logger->debug("zlio::OdometryEstimation initialized");
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

// zlio::Odometry 没有清空接口：上游的"停止"就是析构。这里仅在无并发的
// 停止阶段保留空实现（调用前 worker 已 join，缓冲内容不再变化）。
void OdometryEstimation::stop() {}

PointCloudT::Ptr OdometryEstimation::save_map() {
  if (!param.save_map_en) return nullptr;
  std::lock_guard<std::mutex> lock(scan_queue_mutex);
  return global_map;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  Imu sample;
  sample.time = nanoseconds(imu->stamp);
  sample.angular_velocity = imu->angular_vel;
  sample.acceleration = imu->linear_acc;
  odometry->addImu(sample);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  Scan scan;
  scan.reserve(cloud->size());
  const Time base = nanoseconds(stamp);
  Time last = base;
  for (const auto& point : *cloud) {
    Point echo;
    echo.position = Eigen::Vector3d(point.x, point.y, point.z);
    echo.time = base + nanoseconds(point.curvature * 1e-3);
    last = std::max(last, echo.time);
    scan.push_back(echo);
  }
  odometry->addScan(std::move(scan));

  std::lock_guard<std::mutex> lock(scan_queue_mutex);
  scan_queue.emplace_back(base, last);
}

bool OdometryEstimation::process_once() {
  std::vector<FrameResult> results = odometry->processAvailable(false);
  if (results.empty()) {
    // 没有具备完整惯性覆盖的窗口而回波仍在积压：输入已停止供给（回放尾段或
    // 间歇断流），按上游 flush 语义排空尾段，否则排空等待永远不会归零。
    std::lock_guard<std::mutex> lock(scan_queue_mutex);
    if (!scan_queue.empty()) {
      results = odometry->processAvailable(true);
      if (results.empty()) {
        // 冲刷后仍无产出：输入流已终结，残留扫描包按已消费清算，让排空等待
        // （workload 归零）能够收敛。
        scan_queue.clear();
      }
    }
  }
  if (results.empty()) return false;

  // 用本轮窗口链推进的水位清算已完全消费的扫描包（workload 帧当量）。
  {
    std::lock_guard<std::mutex> lock(scan_queue_mutex);
    watermark = std::max(watermark, results.back().begin);
    while (!scan_queue.empty() && scan_queue.front().second <= watermark)
      scan_queue.pop_front();
  }

  emit_results(results);
  return true;
}

int OdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(scan_queue_mutex);
  return static_cast<int>(scan_queue.size());
}

void OdometryEstimation::emit_results(std::vector<FrameResult>& results) {
  for (auto& result : results) {
    auto frame = std::make_shared<KeyFrame>();
    frame->id = frame_counter++;
    // 契约例外（见 callbacks.hpp）：zlio 无扫描概念，stamp 取本观测窗口的
    // 起点；位姿/速度/偏置是窗口末端的后验，二者不重合。
    frame->stamp = seconds(result.begin);
    frame->frame_id = FrameId::IMU;
    frame->T_world_imu.linear() = result.estimate.state.pose.rotation.toRotationMatrix();
    frame->T_world_imu.translation() = result.estimate.state.pose.position;
    frame->v_world_imu = result.estimate.state.velocity;
    frame->imu_bias.head<3>() = result.estimate.state.gyro_bias;
    frame->imu_bias.tail<3>() = result.estimate.state.accel_bias;

    // 上游回波仅 xyz：云逐帧新分配、发射后不可变；intensity 无源恒 0。
    PointCloudT::Ptr cloud(new PointCloudT());
    cloud->resize(result.cloud.size());
    for (std::size_t i = 0; i < result.cloud.size(); ++i) {
      cloud->points[i].x = static_cast<float>(result.cloud[i].x());
      cloud->points[i].y = static_cast<float>(result.cloud[i].y());
      cloud->points[i].z = static_cast<float>(result.cloud[i].z());
    }
    frame->cloud_imu = cloud;

    if (param.save_map_en && !result.cloud.empty()) {
      // 全量累积语义（上游无地图导出，pcl_wait_save 同款方案）：窗口末端
      // 后验把去畸变回波转世界系逐帧追加，无去重。
      const Eigen::Matrix3d rotation = result.estimate.state.pose.rotation.toRotationMatrix();
      const Eigen::Vector3d& position = result.estimate.state.pose.position;
      for (const auto& point : result.cloud) {
        PointT world;
        const Eigen::Vector3d mapped = rotation * point + position;
        world.x = static_cast<float>(mapped.x());
        world.y = static_cast<float>(mapped.y());
        world.z = static_cast<float>(mapped.z());
        global_map->push_back(world);
      }
    }

    Callbacks::on_new_frame(frame);

    logger->debug(
      "[zlio] window stamp={:.3f} new_points={} matches={} lidar_updated={}",
      seconds(result.begin),
      result.cloud.size(),
      result.matches,
      result.lidar_updated);
  }
}

}  // namespace zlio
}  // namespace asuka
