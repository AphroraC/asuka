/**
 * @file observation.cpp
 * @brief 组装各向异性点噪声与同面共享参数噪声的高斯观测。
 */
#include <asuka/odometry/zlio/observation.hpp>

#include <cmath>
#include <stdexcept>

#include "covariance.hpp"
#include "execution.hpp"
#include "likelihood.hpp"
#include <asuka/odometry/zlio/state.hpp>

namespace asuka {
namespace zlio {
namespace {
using PlaneRow = CompactPlaneRow;

/// 一轮线性化共享状态、地图和点预测协方差；每个查询独立执行。
class PointPlaneMatcher {
public:
    /// 准备一轮关联共享的状态及点预测协方差。
    PointPlaneMatcher(const VoxelMap& map, const State& state, const Mat15& association_covariance)
        : map_(map),
          state_(state),
          rotation_(state.pose.rotation.toRotationMatrix()),
          prediction_(state, association_covariance) {}
    /// 查找匹配平面并计算残差与观测噪声分量。
    void match(const Observation& point, VoxelMap::QueryCache& cache, PlaneRow& output) const;
    /// 计算导航位姿误差下的点面残差 Jacobian。
    Vec6 jacobian(const Observation& point, const PlaneRow& row) const {
        Vec6 h;
        h.head<3>() = row.normal;
        h.tail<3>() = state_.pose.apply(point.point).cross(row.normal);
        return h;
    }

private:
    const VoxelMap& map_;
    const State& state_;
    const Mat3 rotation_;
    const PointCovariance prediction_;
};

/// 结合点预测协方差完成概率关联，并保存观测噪声。
void PointPlaneMatcher::match(const Observation& point, VoxelMap::QueryCache& cache,
                              PlaneRow& output) const {
    output.covariance = nullptr;
    const Vec3 world = state_.pose.apply(point.point);
    const Mat3 association =
        rotation_ * (point.covariance + prediction_(point.point)) * rotation_.transpose();
    const auto match = map_.match(world, association, &cache);
    if (!match)
        return;
    // 将独立测量噪声投影至匹配平面的法向。
    const Vec3 body_normal = rotation_.transpose() * match->normal;
    const double measurement_variance = body_normal.dot(point.covariance * body_normal);
    const double variance = match->plane_variance + match->surface_variance + measurement_variance;
    if (!(variance > 0) || !std::isfinite(variance))
        return;
    auto& constraint = output;
    constraint.normal = match->normal;
    constraint.residual = match->residual;
    constraint.total = variance;
    constraint.independent = match->surface_variance + measurement_variance;
    constraint.plane_jacobian = match->parameter_jacobian;
    constraint.covariance = match->parameter_covariance;
}

/// 按输入顺序执行点面关联并归并共享平面信息。
Linearization linearizeSequential(const std::vector<Observation>& points,
                                  const PointPlaneMatcher& matcher,
                                  std::vector<VoxelMap::QueryCache>& query_cache,
                                  SharedPlaneAccumulator& accumulator) {
    accumulator.reset(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        PlaneRow row;
        matcher.match(points[i], query_cache[i], row);
        if (row.covariance)
            accumulator.add(row, [&] { return matcher.jacobian(points[i], row); });
    }
    return accumulator.finish();
}
}  // namespace

struct PlaneLinearizer::Impl {
    /// 校验关联线程数并创建空的复用工作区。
    explicit Impl(int count) : threads(count) {
        if (count < 1 || count > 2)
            throw std::invalid_argument(
                "point-to-plane association thread count must be one or two");
    }
    int threads;
    std::vector<VoxelMap::QueryCache> queries;
    std::vector<PlaneRow> rows;
    SharedPlaneAccumulator accumulator;
    std::unique_ptr<BatchExecutor> executor;
};

/// 创建固定线程数的点面观测工作区。
PlaneLinearizer::PlaneLinearizer(int threads) : impl_(std::make_unique<Impl>(threads)) {
}
/// 释放关联执行器及临时观测数据。
PlaneLinearizer::~PlaneLinearizer() = default;

/// 开始一次更新并重置逐点根查询缓存。
void PlaneLinearizer::beginUpdate(std::size_t points) {
    impl_->queries.assign(points, VoxelMap::QueryCache{});
}

/// 更新各点关联并以稳定顺序组装共享平面信息。
Linearization PlaneLinearizer::linearize(const std::vector<Observation>& points,
                                         const VoxelMap& map, const State& state,
                                         const Mat15& association_covariance) {
    if (impl_->queries.size() != points.size())
        throw std::logic_error(
            "point-to-plane workspace size does not match the observation point count");
    const PointPlaneMatcher matcher(map, state, association_covariance);
    if (impl_->threads == 1 || points.size() < 512)
        return linearizeSequential(points, matcher, impl_->queries, impl_->accumulator);
    if (!impl_->executor)
        impl_->executor = std::make_unique<BatchExecutor>(2);
    impl_->rows.resize(points.size());
    const auto calculate = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i)
            matcher.match(points[i], impl_->queries[i], impl_->rows[i]);
    };
    const std::size_t middle = points.size() / 2;
    impl_->executor->run(2, [&](std::size_t partition) {
        calculate(partition == 0 ? 0 : middle, partition == 0 ? middle : points.size());
    });
    impl_->accumulator.reset(points.size());
    // 按输入顺序归并并行计算的点面观测。
    for (std::size_t i = 0; i < points.size(); ++i)
        if (impl_->rows[i].covariance)
            impl_->accumulator.add(impl_->rows[i],
                                   [&] { return matcher.jacobian(points[i], impl_->rows[i]); });
    return impl_->accumulator.finish();
}
}  // namespace zlio
}  // namespace asuka
