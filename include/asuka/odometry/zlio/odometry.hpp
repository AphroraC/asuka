/**
 * @file odometry.hpp
 * @brief 协调激光惯性估计的启动、观测处理与后验交接。
 */
#pragma once
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include <asuka/odometry/zlio/config.hpp>
#include <asuka/odometry/zlio/filter.hpp>
#include <asuka/odometry/zlio/initialization.hpp>
#include <asuka/odometry/zlio/input.hpp>
#include <asuka/odometry/zlio/map.hpp>
#include <asuka/odometry/zlio/observation.hpp>
#include <asuka/odometry/zlio/sampling.hpp>
#include <asuka/odometry/zlio/trajectory.hpp>
namespace asuka {
namespace zlio {
class Preparation;
/// 保存观测端点的后验及角速度测量。
struct FrameResult {
    Estimate estimate;
    Vec3 angular_velocity = Vec3::Zero();
    Time begin = 0;
    std::vector<Vec3> cloud;  // 本次新增回波，已去畸变至 estimate 的 IMU 坐标系。
    std::size_t matches = 0;
    bool lidar_updated = false;
};
/// 管理激光惯性估计的完整处理流程。
class Odometry {
public:
    /// 设置算法参数和输入容量。
    explicit Odometry(Config config, InputBufferLimits limits = {});
    /// 释放估计会话与工作区。
    ~Odometry();
    /// 设置独立后端点云分辨率；零表示关闭额外导出。
    void setOutputResolution(double resolution);
    /// 在停止准入并汇合工作线程后重设位置与航向，保留物理速度、重力对齐及 IMU 偏置。
    /// 同步变换协方差并清空地图与观测历史；未初始化或缺少惯性历史时返回 false。
    bool resetWarm();
    /// 接收有序惯性测量并推进初始化。
    void addImu(const Imu& sample);
    /// 接收逐点带时间的雷达回波。
    void addScan(Scan scan);
    /// 处理已具备时间覆盖的观测并按需排空尾段。
    std::vector<FrameResult> processAvailable(bool flush = false);
    /// 返回是否已完成初始化。
    bool initialized() const { return initialized_; }
    /// 返回已提交的导航估计。
    const Estimate& estimate() const { return filter_.estimate(); }
    /// 返回世界系重力向量。
    const Vec3& gravity() const { return gravity_; }
    /// 返回初始化会话的当前状态。
    const InitializationStatus& initializationStatus() const { return initializer_.status(); }
    /// 返回后续后验允许使用的最早时间。
    std::optional<Time> posteriorWatermark() const;

private:
    /// 消费完整启动扫描并尝试生成初始估计。
    void tryInitialize();
    /// 安装初始化后验、地图及处理水位。
    void acceptInitialization(const InitializationResult& result, Time end);
    /// 将导航状态和名义轨迹推进到目标时刻。
    void predictTo(Time time);
    /// 处理单个观测端点并提交估计结果。
    FrameResult process(Time end);
    Config config_;
    Input input_;
    Initializer initializer_;
    ErrorStateFilter filter_;
    Trajectory trajectory_;
    AdaptiveSampler sampler_;
    std::unique_ptr<Preparation> observations_;
    VoxelMap map_;
    PlaneLinearizer linearizer_;
    bool initialized_ = false;
    double output_resolution_ = 0;
    Time initialization_time_ = 0;
    Vec3 gravity_;
};
}  // namespace zlio
}  // namespace asuka
