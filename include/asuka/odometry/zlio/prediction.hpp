/**
 * @file prediction.hpp
 * @brief 接收已提交后验并维护独立的惯性预测状态。
 */
#pragma once
#include <optional>
#include <utility>

#include <asuka/odometry/zlio/filter.hpp>
#include <asuka/odometry/zlio/input.hpp>
namespace asuka {
namespace zlio {
/// 根据已提交后验维护独立的惯性预测状态。
class OutputPredictor {
public:
    /// 配置固定世界重力、噪声及可接受的后验迟到范围。
    OutputPredictor(Vec3 gravity, ImuNoise noise, ImuHistoryLimits limits = {})
        : imus_(limits), gravity_(std::move(gravity)), noise_(noise) {}
    /// 保存真实 IMU 样本，初始化后返回同采样时刻的预测结果。
    std::optional<Estimate> addImu(const Imu& sample);
    /// 提交完整后验并重传播；已被覆盖的提交返回 false，历史缺失则抛出异常。
    bool correct(const Estimate& posterior, std::optional<Vec3> gravity = std::nullopt);
    /// 强制安装新规范的后验并重传播原始 IMU，允许在同一时刻重置；历史须覆盖该时刻。
    void reset(const Estimate& posterior, std::optional<Vec3> gravity = std::nullopt);
    /// 判断预测器是否已经接受过一个完整后验。
    bool initialized() const { return initialized_; }
    /// 清退历史；调用方保证不再提交更早后验，等于水位的提交仍可接受。
    void discardBefore(Time watermark) { imus_.discardBefore(watermark); }
    /// 判断历史能否支撑指定时刻的重置及后续重传播。
    bool canReset(Time time) const { return imus_.covers(time); }

private:
    ErrorStateFilter predictor_;
    ImuHistory imus_;
    Vec3 gravity_;
    ImuNoise noise_;
    bool initialized_ = false;
    Time posterior_time_ = 0;
};

}  // namespace zlio
}  // namespace asuka
