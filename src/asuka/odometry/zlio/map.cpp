/**
 * @file map.cpp
 * @brief 更新稀疏平面地图并按概率评分关联查询点。
 */
#include <asuka/odometry/zlio/map.hpp>

#include <algorithm>
#include <atomic>
#include <limits>
#include <stdexcept>

#include "storage.hpp"
#include <asuka/odometry/zlio/state.hpp>

namespace asuka {
namespace zlio {
namespace {
/// 借用批次位姿协方差，保存单个入图点的世界系几何量。
struct MapPointView {
    Vec3 point;
    Mat3 independent_covariance;
    const MapPoseCovariance& shared_pose_covariance;
    /// 将批次位姿协方差传播为当前点的边缘协方差。
    Mat3 marginalCovariance() const {
        const auto J = pointPoseJacobian(point);
        return J * (*shared_pose_covariance) * J.transpose() + independent_covariance;
    }
};
/// 以单精度体素坐标和负向整数映射计算地图根键。
VoxelKey keyFor(const Vec3& point, double size) {
    std::array<std::int64_t, 3> out;
    for (int j = 0; j < 3; ++j) {
        float q = point[j] / size;
        if (q < 0)
            q -= 1.;
        out[j] = static_cast<std::int64_t>(q);
    }
    return {out[0], out[1], out[2]};
}
}  // namespace
struct VoxelMap::Impl {
    using Cell = RootIndex::Cell;
    MapConfig config;
    RootIndex index;
    std::size_t revision = 0;
    bool initialized = false;
    const std::uint64_t identity;
    /// 创建地图根存储并分配独立缓存标识。
    explicit Impl(MapConfig c) : config(std::move(c)), identity(nextIdentity()) {}
    /// 从批量种子点建立所有根节点的平面树。
    void initialize() {
        for (auto& c : index)
            c.node.initialize(voxelCenter(c.key, config.voxel_size), config.voxel_size * .5, 0);
        initialized = true;
    }
    /// 为地图实例分配单调递增的缓存标识。
    static std::uint64_t nextIdentity() {
        static std::atomic<std::uint64_t> sequence{1};
        return sequence.fetch_add(1, std::memory_order_relaxed);
    }
    /// 在根数量限制内创建新的体素根节点。
    Cell& create(const VoxelKey& key) {
        if (index.size() >= config.maximum_roots)
            throw std::runtime_error("moment map exceeds root resource limit");
        return index.insert(key);
    }

    /// 优先使用当前地图版本的根节点查询缓存。
    const Cell* find(const VoxelKey& key, QueryCache* cache) const {
        if (cache) {
            if (cache->owner_id_ != identity || cache->revision_ != revision) {
                cache->ready_.fill(false);
                cache->next_ = 0;
                cache->owner_id_ = identity;
                cache->revision_ = revision;
            }
            for (std::size_t i = 0; i < cache->keys_.size(); ++i)
                if (cache->ready_[i] && cache->keys_[i] == key)
                    return static_cast<const Cell*>(cache->entries_[i]);
        }
        const Cell* result = index.find(key);
        if (cache) {
            const auto slot = cache->next_++ % cache->keys_.size();
            cache->keys_[slot] = key;
            cache->entries_[slot] = result;
            cache->ready_[slot] = true;
        }
        return result;
    }
};
/// 创建累计平面地图及根索引。
VoxelMap::VoxelMap(MapConfig config) : impl_(std::make_unique<Impl>(config)) {
}
/// 释放地图全部统计量、支持点及查询索引。
VoxelMap::~VoxelMap() = default;
/// 重建空地图并更换查询缓存身份。
void VoxelMap::clear() {
    impl_ = std::make_unique<Impl>(impl_->config);
}
/// 按输入顺序插入已处于世界系的支持点。
void VoxelMap::insert(const std::vector<MapPoint>& points) {
    if (points.empty())
        return;
    // 推进地图版本，使已保存的查询缓存重新定位根节点。
    ++impl_->revision;
    auto add = [&](const MapPoint& point, bool seed) {
        const auto key = keyFor(point.point, impl_->config.voxel_size);
        const auto found = impl_->index.find(key);
        auto& cell = found ? *found : impl_->create(key);
        if (seed)
            cell.node.seed(point);
        else
            cell.node.update(point, voxelCenter(key, impl_->config.voxel_size),
                             impl_->config.voxel_size * .5, 0);
    };
    if (!impl_->initialized) {
        for (const auto& p : points)
            add(p, true);
        impl_->initialize();
    } else {
        for (const auto& point : points)
            add(point, false);
    }
}
/// 逐点变换观测并更新地图，共享当前后验的位姿协方差。
void VoxelMap::insert(const std::vector<Observation>& points, const Estimate& estimate) {
    if (points.empty())
        return;
    ++impl_->revision;
    const Mat3 rotation = estimate.state.pose.rotation.toRotationMatrix();
    const MapPoseCovariance shared(estimate.covariance.topLeftCorner<6, 6>());
    for (const auto& point : points) {
        const Vec3 world = estimate.state.pose.apply(point.point);
        const auto key = keyFor(world, impl_->config.voxel_size);
        const auto found = impl_->index.find(key);
        auto& cell = found ? *found : impl_->create(key);
        const auto materialize = [&]() {
            return MapPointView{world, rotation * point.covariance * rotation.transpose(), shared};
        };
        if (!impl_->initialized)
            cell.node.seed(materialize());
        else
            cell.node.update(world, materialize, voxelCenter(key, impl_->config.voxel_size),
                             impl_->config.voxel_size * .5, 0);
    }
    if (!impl_->initialized)
        impl_->initialize();
}
/// 结合查询点边缘协方差搜索本格及邻格的最佳平面。
std::optional<PlaneMatch> VoxelMap::match(const Vec3& point, const Mat3& query_covariance,
                                          QueryCache* cache) const {
    const double size = impl_->config.voxel_size;
    const auto key = keyFor(point, size);
    std::optional<PlaneMatch> best;
    double score = -std::numeric_limits<double>::infinity();
    auto visit = [&](const Impl::Cell* cell) {
        if (!cell)
            return;
        cell->node.visit([&](const plane::Fit& surface) {
            const Vec3 offset = point - surface.center;
            const double residual = surface.normal.dot(offset);
            const double tangent2 = std::max(0., offset.squaredNorm() - residual * residual);
            if (tangent2 > 9 * surface.radius * surface.radius)
                return;
            Vec3 plane_jacobian;
            const double plane_variance = surface.variance(point, plane_jacobian);
            if (!std::isfinite(plane_variance))
                return;
            const double variance = plane_variance + surface.surface_variance +
                                    surface.normal.dot(query_covariance * surface.normal);
            if (!(variance > 0) || !std::isfinite(variance) || residual * residual >= 9 * variance)
                return;
            const double current = -.5 * (std::log(variance) + residual * residual / variance);
            if (current > score) {
                score = current;
                best =
                    PlaneMatch{surface.normal, residual, plane_variance, surface.surface_variance};
                best->parameter_jacobian = plane_jacobian;
                best->parameter_covariance = &surface.noise;
            }
        });
    };
    // 本根匹配成功即返回；否则遍历边界半径涉及的面、边及角相邻根。
    visit(impl_->find(key, cache));
    if (best)
        return best;
    const Vec3 center = voxelCenter(key, size);
    std::array<int, 3> direction{};
    for (int j = 0; j < 3; ++j) {
        if (point[j] > center[j] + size * .25)
            direction[j] = 1;
        else if (point[j] < center[j] - size * .25)
            direction[j] = -1;
    }
    for (int x = 0; x <= (direction[0] != 0); ++x)
        for (int y = 0; y <= (direction[1] != 0); ++y)
            for (int z = 0; z <= (direction[2] != 0); ++z) {
                if (x == 0 && y == 0 && z == 0)
                    continue;
                const VoxelKey near{key.x + x * direction[0], key.y + y * direction[1],
                                    key.z + z * direction[2]};
                visit(impl_->find(near, cache));
            }
    return best;
}
/// 返回当前累计根体素数量。
std::size_t VoxelMap::size() const {
    return impl_->index.size();
}
}  // namespace zlio
}  // namespace asuka
