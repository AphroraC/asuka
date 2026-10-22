/**
 * @file storage.hpp
 * @brief 管理地图累计统计、体素树表示及地址稳定的根索引。
 */
#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "moments.hpp"
#include <asuka/odometry/zlio/voxel.hpp>

namespace asuka {
namespace zlio::plane {
/// 累计地图点矩，并分别保存独立传感器噪声。
class MapStatistics {
    MomentStatistics marginal_;
    Mat3 sensor_sum_ = Mat3::Zero();
    bool has_pose_source_ = false;

public:
    /// 累加点边缘噪声及独立测量噪声。
    template <class Point>
    void add(const Point& point, const Vec3& origin) {
        marginal_.add(point, origin);
        sensor_sum_ += point.independent_covariance;
        has_pose_source_ = has_pose_source_ || bool(point.shared_pose_covariance);
    }
    /// 由累计统计拟合平面及残差不确定性。
    Fit fit(const Vec3& origin) const {
        return marginal_.fit(origin, .01, has_pose_source_ ? &sensor_sum_ : nullptr);
    }
};

/// 地图细分树的最大深度。
inline constexpr int kMaximumDepth = 4;
/// 紧凑保存尚未形成平面的支持点。
struct BootstrapPoint {
    Vec3 point;
    std::array<double, 6> covariance;
    MapPoseCovariance shared_pose_covariance;
    /// 压缩保存支持点的对称测量协方差。
    template <class Point>
    BootstrapPoint(const Point& value)
        : point(value.point), shared_pose_covariance(value.shared_pose_covariance) {
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            covariance[k] = .5 * (value.independent_covariance(ij[0], ij[1]) +
                                  value.independent_covariance(ij[1], ij[0]));
        }
    }
    /// 恢复完整测量协方差及共享位姿引用。
    MapPoint expand() const {
        MapPoint value;
        value.point = point;
        value.shared_pose_covariance = shared_pose_covariance;
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            value.independent_covariance(ij[0], ij[1]) =
                value.independent_covariance(ij[1], ij[0]) = covariance[k];
        }
        return value;
    }
};
static_assert(sizeof(BootstrapPoint) == 9 * sizeof(double) + sizeof(void*),
              "XYZ, full symmetric sensor covariance and one shared-pose handle required");

/// 根据支持数量与平面形状管理叶节点、冻结平面和子树。
class Node {
    enum class Kind : std::uint8_t { Bootstrap, Active, Frozen, Branch };
    struct Active {
        MapStatistics moments;
        Fit plane;
    };
    struct Branch {
        std::array<std::unique_ptr<Node>, 8> child;
    };
    Kind kind_ = Kind::Bootstrap;
    std::uint16_t seen_ = 0;
    std::uint8_t pending_ = 0;
    union Storage {
        std::vector<BootstrapPoint>* bootstrap;
        Active* active;
        Fit* frozen;
        Branch* branch;
        /// 创建初始支持点容器。
        Storage() : bootstrap(new std::vector<BootstrapPoint>) {}
    } value_;
    static constexpr int kMaximumUpdates = 1000;
    /// 按点相对中心的位置计算八叉树子节点编号。
    static int octant(const Vec3& point, const Vec3& center) {
        return (point.x() > center.x() ? 4 : 0) + (point.y() > center.y() ? 2 : 0) +
               (point.z() > center.z() ? 1 : 0);
    }
    /// 根据子节点编号计算子体素中心。
    static Vec3 childCenter(const Vec3& center, double half, int index) {
        return center +
               half * .5 * Vec3(index & 4 ? 1 : -1, index & 2 ? 1 : -1, index & 1 ? 1 : -1);
    }
    /// 根据当前节点类型释放所持表示。
    void destroy() {
        switch (kind_) {
            case Kind::Bootstrap:
                delete value_.bootstrap;
                break;
            case Kind::Active:
                delete value_.active;
                break;
            case Kind::Frozen:
                delete value_.frozen;
                break;
            case Kind::Branch:
                delete value_.branch;
                break;
        }
    }
    /// 将活动统计转换为只读平面表示。
    void freeze() {
        auto replacement = std::make_unique<Fit>(value_.active->plane);
        destroy();
        value_.frozen = replacement.release();
        kind_ = Kind::Frozen;
    }

public:
    /// 创建等待初始支持点的节点。
    Node() = default;
    /// 释放节点表示及其拥有的子树。
    ~Node() { destroy(); }
    /// 禁止复制节点所有权。
    Node(const Node&) = delete;
    /// 禁止复制节点所有权。
    Node& operator=(const Node&) = delete;
    /// 向初始支持点容器追加一个点。
    template <class Point>
    void seed(const Point& point) {
        value_.bootstrap->push_back(point);
    }
    /// 根据累计初始支持点建立平面或递归细分。
    void initialize(const Vec3& center, double half, int depth, bool initial_freeze = true) {
        if (kind_ != Kind::Bootstrap)
            return;
        auto* raw = value_.bootstrap;
        if (raw->size() <= 5)
            return;
        // 建立初始统计并拟合当前层平面。
        auto active = std::make_unique<Active>();
        for (const auto& point : *raw)
            active->moments.add(point.expand(), center);
        active->plane = active->moments.fit(center);
        pending_ = 0;
        if (!active->plane.valid && depth < kMaximumDepth) {
            auto branch = std::make_unique<Branch>();
            for (const auto& point : *raw) {
                int index = octant(point.point, center);
                auto& c = branch->child[index];
                if (!c)
                    c = std::make_unique<Node>();
                c->seed(point);
            }
            for (int k = 0; k < 8; ++k)
                if (branch->child[k])
                    branch->child[k]->initialize(childCenter(center, half, k), half * .5, depth + 1,
                                                 false);
            destroy();
            value_.branch = branch.release();
            kind_ = Kind::Branch;
        } else {
            const bool frozen =
                initial_freeze && active->plane.valid && raw->size() > kMaximumUpdates;
            destroy();
            value_.active = active.release();
            kind_ = Kind::Active;
            if (frozen)
                freeze();
        }
    }
    /// 根据支持点更新节点的累计平面统计。
    void update(const MapPoint& point, const Vec3& center, double half, int depth) {
        update(
            point.point, [&]() -> const MapPoint& { return point; }, center, half, depth);
    }
    /// 沿树定位叶节点，并在接纳支持点时构造其协方差。
    template <class Materialize>
    void update(const Vec3& position, const Materialize& materialize, const Vec3& center,
                double half, int depth) {
        if (kind_ == Kind::Frozen)
            return;
        if (kind_ == Kind::Branch) {
            const int index = octant(position, center);
            auto& c = value_.branch->child[index];
            if (!c)
                c = std::make_unique<Node>();
            c->update(position, materialize, childCenter(center, half, index), half * .5,
                      depth + 1);
            return;
        }
        ++seen_;
        if (kind_ == Kind::Bootstrap) {
            seed(materialize());
            initialize(center, half, depth);
            return;
        }
        const auto& support = materialize();
        auto* active = value_.active;
        active->moments.add(support, center);
        if (++pending_ > 5) {
            active->plane = active->moments.fit(center);
            pending_ = 0;
            if (!active->plane.valid && depth < kMaximumDepth) {
                // 历史压缩矩不能按子体素拆分；以本次支持点开始建立子节点。
                auto replacement = std::make_unique<Branch>();
                const int index = octant(position, center);
                auto& child = replacement->child[index];
                child = std::make_unique<Node>();
                child->update(MapPoint{support.point, support.independent_covariance,
                                       support.shared_pose_covariance},
                              childCenter(center, half, index), half * .5, depth + 1);
                destroy();
                value_.branch = replacement.release();
                kind_ = Kind::Branch;
                return;
            }
        }
        // 达到更新数量后冻结最近一次拟合的平面。
        if (seen_ >= kMaximumUpdates)
            freeze();
    }
    /// 按子节点顺序访问全部有效平面。
    template <class Visitor>
    void visit(const Visitor& visitor) const {
        if (kind_ == Kind::Active) {
            const auto& p = value_.active->plane;
            if (p.valid)
                visitor(p);
        } else if (kind_ == Kind::Frozen) {
            const auto& p = *value_.frozen;
            if (p.valid)
                visitor(p);
        } else if (kind_ == Kind::Branch)
            for (const auto& c : value_.branch->child)
                if (c)
                    c->visit(visitor);
    }
};
}  // namespace zlio::plane
}  // namespace asuka

namespace asuka {
namespace zlio {

/// 使用连续编号索引地址稳定的地图根节点。
class RootIndex {
public:
    /// 保存根体素键及其平面树。
    struct Cell {
        VoxelKey key;
        plane::Node node;
        /// 创建指定体素的空根节点。
        explicit Cell(const VoxelKey& value) : key(value) {}
    };
    /// 返回根节点数量。
    std::size_t size() const { return cells_.size(); }
    /// 返回根节点遍历起点。
    auto begin() { return cells_.begin(); }
    /// 返回根节点遍历终点。
    auto end() { return cells_.end(); }
    /// 返回只读根节点遍历起点。
    auto begin() const { return cells_.begin(); }
    /// 返回只读根节点遍历终点。
    auto end() const { return cells_.end(); }

    /// 查询指定体素的根节点，未找到时返回空指针。
    const Cell* find(const VoxelKey& key) const {
        if (buckets_.empty())
            return nullptr;
        const auto hash = mixed(key);
        const auto tag = fingerprint(hash);
        auto at = hash & (buckets_.size() - 1);
        while (tags_[at]) {
            const auto id = buckets_[at];
            if (tags_[at] == tag && cells_[id].key == key)
                return &cells_[id];
            at = (at + 1) & (buckets_.size() - 1);
        }
        return nullptr;
    }
    /// 查询可修改的根节点，未找到时返回空指针。
    Cell* find(const VoxelKey& key) {
        return const_cast<Cell*>(static_cast<const RootIndex&>(*this).find(key));
    }
    /// 获取指定根节点，不存在时分配节点并登记索引。
    Cell& insert(const VoxelKey& key) {
        if (auto* old = find(key))
            return *old;
        if (cells_.size() >= std::numeric_limits<std::uint32_t>::max())
            throw std::length_error("root ID capacity exceeded");
        if (buckets_.empty() || cells_.size() + 1 > buckets_.size() - buckets_.size() / 4)
            rehash(buckets_.empty() ? 16 : buckets_.size() * 2);
        const auto hash = mixed(key);
        auto at = hash & (buckets_.size() - 1);
        while (tags_[at])
            at = (at + 1) & (buckets_.size() - 1);
        const auto id = cells_.size();
        cells_.emplace_back(key);
        buckets_[at] = static_cast<std::uint32_t>(id);
        tags_[at] = fingerprint(hash);
        return cells_.back();
    }

private:
    std::deque<Cell> cells_;
    std::vector<std::uint32_t> buckets_;
    std::vector<std::uint8_t> tags_;
    /// 混合体素键的高低位，减少规则空间坐标的碰撞。
    static std::uint64_t mixed(const VoxelKey& key) {
        std::uint64_t x = VoxelHash{}(key);
        x ^= x >> 30;
        x *= 0xbf58476d1ce4e5b9ULL;
        x ^= x >> 27;
        x *= 0x94d049bb133111ebULL;
        return x ^ (x >> 31);
    }
    /// 提取非零短指纹，零值表示空桶。
    static std::uint8_t fingerprint(std::uint64_t x) { return (x >> 56) | 1; }
    /// 扩展哈希桶并重新登记根编号。
    void rehash(std::size_t capacity) {
        if (capacity <= buckets_.size())
            throw std::length_error("root index capacity overflow");
        std::vector<std::uint32_t> buckets(capacity);
        std::vector<std::uint8_t> tags(capacity);
        for (std::size_t id = 0; id < cells_.size(); ++id) {
            auto hash = mixed(cells_[id].key);
            auto at = hash & (capacity - 1);
            while (tags[at])
                at = (at + 1) & (capacity - 1);
            buckets[at] = id;
            tags[at] = fingerprint(hash);
        }
        buckets_.swap(buckets);
        tags_.swap(tags);
    }
};
}  // namespace zlio
}  // namespace asuka
