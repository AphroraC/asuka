/**
 * @file filter.hpp
 * @brief 管理十五维导航后验及迭代校正。
 */
#pragma once

#include <functional>
#include <optional>
#include <utility>

#include <asuka/odometry/zlio/inertial.hpp>

namespace asuka {
namespace zlio {

/// 表示本次观测是否提交了后验状态与协方差。
struct UpdateResult {
    bool accepted = false;
};

/// 独占十五维导航估计；一次观测的全部迭代共用同一传播先验。
class ErrorStateFilter {
public:
    /// 构造尚未载入外部初始估计的滤波器。
    ErrorStateFilter() = default;
    /// 载入状态及其导航误差协方差；协方差非数值正定时抛出异常。
    void reset(const State& state, const Mat15& covariance);
    /// 返回当前导航状态的只读引用，有效期截至下一次状态更新。
    const State& state() const { return estimate_.state; }
    /// 返回当前状态局部误差坐标中的完整十五维协方差。
    const Mat15& covariance() const { return estimate_.covariance; }
    /// 返回当前状态与协方差，不转移其所有权。
    const Estimate& estimate() const { return estimate_; }
    /// 沿有序 IMU 区间传播，并返回拥有独立插值参数的名义轨迹段。
    ImuSegment predict(const Imu& left, const Imu& right, const Vec3& gravity,
                       const ImuNoise& noise);
    /// 每轮回调接收当前线性化状态及该图表下的先验协方差；只提交一次后验。
    UpdateResult update(const std::function<Linearization(const State&, const Mat15&)>& linearize,
                        int maximum_iterations = 5, double tolerance = 0.001);

private:
    Estimate estimate_;
};

}  // namespace zlio
}  // namespace asuka
