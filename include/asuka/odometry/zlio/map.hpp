/**
 * @file map.hpp
 * @brief 管理累计平面地图并提供概率点面关联接口。
 */
#pragma once

#include <memory>
#include <optional>

#include <asuka/odometry/zlio/config.hpp>
#include <asuka/odometry/zlio/plane.hpp>
#include <asuka/odometry/zlio/voxel.hpp>

namespace asuka {
namespace zlio {

/// 稀疏哈希地图保存累计概率平面。
class VoxelMap {
    struct Impl;

public:
    /// 每个观测独占根节点地址缓存；不缓存匹配结果或残差。
    class QueryCache {
        friend class VoxelMap;
        friend struct Impl;
        std::uint64_t owner_id_ = 0;
        std::size_t revision_ = 0, next_ = 0;
        // 缓存最近定位的根节点地址；每次都重算残差与方差。
        std::array<VoxelKey, 4> keys_{};
        std::array<const void*, 4> entries_{};
        std::array<bool, 4> ready_{};
    };
    /// 创建稀疏体素索引及累计平面存储。
    explicit VoxelMap(MapConfig config = {});
    /// 释放全部拥有的点、平面与索引。
    ~VoxelMap();
    /// 禁止复制点与缓存的唯一所有权。
    VoxelMap(const VoxelMap&) = delete;
    /// 禁止复制点与缓存的唯一所有权。
    VoxelMap& operator=(const VoxelMap&) = delete;
    /// 按输入点流顺序更新累计统计量；运行调用方按获取时间提交新点。
    void insert(const std::vector<MapPoint>& points);
    /// 借用同一后验按原点序入图；仅持久化的原始支持点取得共同位姿所有权。
    void insert(const std::vector<Observation>& points, const Estimate& estimate);
    /// 根据世界系查询点及其预测边缘协方差执行概率关联。
    std::optional<PlaneMatch> match(const Vec3& point, const Mat3& query_covariance,
                                    QueryCache* cache = nullptr) const;
    /// 返回当前保留的格数量。
    std::size_t size() const;
    /// 释放局部地图并更新缓存身份。
    void clear();

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace zlio
}  // namespace asuka
