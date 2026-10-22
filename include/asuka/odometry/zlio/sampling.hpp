/**
 * @file sampling.hpp
 * @brief 定义观测时间选择及自适应体素采样接口。
 */
#pragma once

#include <deque>
#include <memory>
#include <vector>

#include <asuka/odometry/zlio/config.hpp>

namespace asuka {
namespace zlio {

/// 根据机体系运动曲率及点面信息选择下一观测时长，返回单位为秒。
double observationInterval(const State& state, const Mat6& information, std::size_t matches,
                           const Vec6& curvature, const WindowConfig& config);

/// 参考雷达系去畸变坐标与真实时刻，代表点选择不依赖协方差。
struct SamplingPoint {
    Vec3 point = Vec3::Zero();
    Time time = 0;
};

/// 两个按原输入次序排列的索引流，均由返回值独立拥有。
struct SampledIndices {
    std::vector<std::size_t> update, merge;
};

/// 独占密度控制历史和三张可复用体素表，不共享可变工作区。
class AdaptiveSampler {
public:
    /// 按配置建立采样控制器。
    explicit AdaptiveSampler(SamplingConfig config = {});
    /// 释放采样历史与工作区。
    ~AdaptiveSampler();
    /// 禁止复制控制历史与独占采样工作区。
    AdaptiveSampler(const AdaptiveSampler& other) = delete;
    /// 禁止复制控制历史与独占采样工作区。
    AdaptiveSampler& operator=(const AdaptiveSampler& other) = delete;
    /// 转移控制历史和工作区的唯一所有权。
    AdaptiveSampler(AdaptiveSampler&& other);
    /// 接管另一采样器的状态与工作区，替换当前内容。
    AdaptiveSampler& operator=(AdaptiveSampler&& other);
    /// 返回稳定采样索引；更新与入图只含 time>after，零 interval 不推进控制器。
    SampledIndices sampleIndices(const std::vector<SamplingPoint>& input, double interval,
                                 Time after);

private:
    /// 根据全窗口代表点距离与点数误差更新采样体素尺寸。
    void updateSize(std::vector<double> ranges, double interval);
    SamplingConfig config_;
    std::deque<double> medians_;
    double size_, previous_error_ = 0;
    bool first_ = true;
    struct Workspace;
    std::unique_ptr<Workspace> workspace_;
};

}  // namespace zlio
}  // namespace asuka
