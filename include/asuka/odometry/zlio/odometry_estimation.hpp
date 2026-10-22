#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/odometry_estimation.hpp>
#include <asuka/core/types.hpp>
#include <asuka/odometry/zlio/cloud_preprocess.hpp>
#include <asuka/odometry/zlio/config.hpp>
#include <asuka/odometry/zlio/odometry.hpp>
#include <asuka/odometry/zlio/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace zlio {

// 插件层参数。算法参数全量保存在 zlio::Config（构造期经 loadConfig 从
// config_zlio.json 读入，键结构与上游 lio.yaml 同构），不在此重复。
struct OdometryEstimationParam {
  bool save_map_en{false};
  double output_resolution{0.1};  // KeyFrame/存图消费的导出点云分辨率（m，0=关闭）
};

// zlio 前端：15 维半直积误差状态滤波 + 连续点流自适应窗口（TDT z-localization
// 的 ROS-free 前端逐位移植）。上游以"观测窗口"而非扫描为更新单位，on_new_frame
// 按窗口节奏发射；KeyFrame::stamp 取窗口起点（zlio 例外条款，见 callbacks.hpp）。
class OdometryEstimation : public asuka::OdometryEstimation {
public:
  OdometryEstimation();
  ~OdometryEstimation() override;

  OdometryEstimation(const OdometryEstimation&) = delete;
  OdometryEstimation& operator=(const OdometryEstimation&) = delete;

  void stop() override;
  PointCloudT::Ptr save_map() override;

  void insert_imu(const ImuData::ConstPtr& imu) override;
  void insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) override;
  void insert_image(const ImageData::ConstPtr& image) override {}

  bool process_once() override;
  int workload() const override;

  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess() override { return cloud_preprocess_impl; }

private:
  void emit_results(std::vector<FrameResult>& results);

  OdometryEstimationParam param;
  Config config;
  std::unique_ptr<Odometry> odometry{nullptr};
  std::shared_ptr<asuka::CloudPreprocess> cloud_preprocess_impl{nullptr};

  PointCloudT::Ptr global_map{new PointCloudT};
  long frame_counter{0};

  // 已插入未消费的扫描包时间跨度（worker 写、workload() 跨线程读）
  mutable std::mutex scan_queue_mutex;
  std::deque<std::pair<Time, Time>> scan_queue;
  Time watermark{0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace zlio
}  // namespace asuka
