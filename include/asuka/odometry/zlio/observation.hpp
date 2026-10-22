/**
 * @file observation.hpp
 * @brief 定义共享平面观测与点面关联工作区接口。
 */
#pragma once

#include <memory>

#include <asuka/odometry/zlio/map.hpp>

namespace asuka {
namespace zlio {

/// 由单个调用线程拥有，复用查询缓存并按原序归并并行计算结果。
class PlaneLinearizer {
public:
    /// 设置包含调用线程在内的关联线程数，仅允许一或二。
    explicit PlaneLinearizer(int threads = 1);
    /// 等待辅助任务结束后释放工作区和地图借用记录。
    ~PlaneLinearizer();
    /// 禁止复制执行器与可变查询工作区。
    PlaneLinearizer(const PlaneLinearizer&) = delete;
    /// 禁止复制执行器与可变查询工作区。
    PlaneLinearizer& operator=(const PlaneLinearizer&) = delete;
    /// 开始新的观测更新并清空上一次更新的关联及邻格借用。
    void beginUpdate(std::size_t points);
    /// 每轮更新关联及残差，返回按原输入顺序累加的概率点面观测。
    Linearization linearize(const std::vector<Observation>& points, const VoxelMap& map,
                            const State& state, const Mat15& association_covariance);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace zlio
}  // namespace asuka
