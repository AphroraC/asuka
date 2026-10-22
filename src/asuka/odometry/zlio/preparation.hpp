/**
 * @file preparation.hpp
 * @brief 根据逐点轨迹准备观测与采样反馈几何。
 */
#pragma once
#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <unordered_set>

#include "covariance.hpp"
#include <asuka/odometry/zlio/input.hpp>
#include <asuka/odometry/zlio/sampling.hpp>
#include <asuka/odometry/zlio/trajectory.hpp>
#include <asuka/odometry/zlio/voxel.hpp>
namespace asuka {
namespace zlio {
/// 保存采样反馈使用的历史回波坐标。
class GeometryCache {
    struct Entry {
        Vec3 point;
        std::uint64_t id = 0;
        bool valid = false;
    };
    struct Pending {
        Vec3 point;
        std::uint64_t id;
    };
    std::vector<Entry> entries_;
    std::vector<Pending> pending_;
    Time consumed_ = 0, end_ = 0;
    static constexpr std::size_t kSlots = 32768, kMaximumPending = 65536;

    /// 保存指定回波的世界坐标。
    void store(std::uint64_t id, const Vec3& point) {
        if (entries_.empty())
            entries_.resize(kSlots);
        entries_[id % kSlots] = {point, id, true};
    }

public:
    /// 开始准备新的观测时间片。
    void begin(Time consumed, Time end) {
        consumed_ = consumed;
        end_ = end;
        pending_.clear();
    }
    /// 查找已消费回波的世界坐标。
    const Vec3* find(std::uint64_t id, Time time) const {
        if (time >= consumed_ || entries_.empty())
            return nullptr;
        const auto& entry = entries_[id % kSlots];
        return entry.valid && entry.id == id ? &entry.point : nullptr;
    }
    /// 保存历史坐标或暂存本次新增坐标。
    void remember(std::uint64_t id, Time time, const Vec3& world) {
        if (time < consumed_)
            store(id, world);
        else if (time > consumed_ && time < end_ && pending_.size() < kMaximumPending)
            pending_.push_back({world, id});
    }
    /// 按已提交位姿校正暂存坐标。
    void commit(const Pose* correction) {
        for (const auto& point : pending_)
            store(point.id, correction ? correction->apply(point.point) : point.point);
        pending_.clear();
    }
};

/// 组织轨迹投影、采样选择与点观测构造。
class Preparation {
public:
    /// 物化后的观测由前端独占；采样器只返回索引，不复制完整观测。
    struct SampledCloud {
        std::vector<Observation> update, merge;
        std::vector<Vec3> output;
    };
    /// 保存一次观测准备所需的时间边界。
    struct Timing {
        Time initialized, consumed;
    };

private:
    /// 保存原始回波在世界系的位置与获取时刻旋转。
    struct WorldPoint {
        Vec3 point;
        Eigen::Quaterniond rotation;
    };
    /// 保存递增时间查询的轨迹游标。
    struct Projection {
        const State& reference;
        Trajectory::Iterator segment, end;
    };
    /// 保存参考雷达系的窗口几何和原始回波视图。
    struct Geometry {
        WindowPointRange points;
        std::size_t first;
        std::size_t first_new;
        Eigen::Quaterniond reference_inverse;
        std::vector<SamplingPoint> sampled;
        std::vector<Eigen::Quaterniond> rotations;
    };

public:
    /// 配置观测外参和测量噪声。
    explicit Preparation(const Config& config)
        : extrinsic_(config.body_from_lidar),
          noise_(config.map.range_sigma, config.map.angle_sigma_degrees) {}

private:
    /// 获取回波的世界几何，并复用已提交时段的缓存。
    WorldPoint project(const Point& point, Projection& context) {
        if (const Vec3* cached = cache_.find(point.id, point.time))
            return {*cached, Eigen::Quaterniond::Identity()};
        Pose pose;
        if (point.time == context.reference.time)
            pose = context.reference.pose;
        else {
            while (context.segment != context.end && context.segment->end().time < point.time)
                ++context.segment;
            if (context.segment == context.end)
                throw std::out_of_range("inertial trajectory does not cover the return timestamp");
            pose = context.segment->at(point.time);
        }
        WorldPoint result{pose.apply(extrinsic_.apply(point.position)), pose.rotation};
        cache_.remember(point.id, point.time, result.point);
        return result;
    }

    /// 将窗口回波投影到当前参考雷达坐标系。
    Geometry geometry(WindowPointRange points, const State& reference, const Trajectory& trajectory,
                      const Timing& timing) {
        assert(std::is_sorted(points.begin(), points.end(),
                              [](const WindowPoint& a, const WindowPoint& b) {
                                  return a.point.time < b.point.time;
                              }));
        const auto first = std::lower_bound(
            points.begin(), points.end(), timing.initialized,
            [](const WindowPoint& point, Time time) { return point.point.time < time; });
        Geometry result{
            points,
            static_cast<std::size_t>(first - points.begin()),
            static_cast<std::size_t>(std::upper_bound(first, points.end(), timing.consumed,
                                                      [](Time time, const WindowPoint& point) {
                                                          return time < point.point.time;
                                                      }) -
                                     first),
            reference.pose.rotation.conjugate(),
            {},
            {}};
        result.sampled.reserve(points.size() - result.first);
        result.rotations.reserve(points.size() - result.first - result.first_new);
        Projection context{reference, trajectory.begin(), trajectory.end()};
        for (std::size_t index = result.first; index < points.size(); ++index) {
            const auto& point = points[index].point;
            const WorldPoint world = project(point, context);
            const Vec3 body = reference.pose.inverseApply(world.point);
            result.sampled.push_back({extrinsic_.inverseApply(body), point.time});
            if (index - result.first >= result.first_new)
                result.rotations.push_back(world.rotation);
        }
        return result;
    }

    /// 构造一个参考 IMU 系观测及其测量协方差。
    Observation observation(const Geometry& geometry, std::size_t index) const {
        const auto& point = geometry.points[geometry.first + index].point;
        assert(index >= geometry.first_new);
        const Mat3 rotation = (geometry.reference_inverse *
                               geometry.rotations[index - geometry.first_new] * extrinsic_.rotation)
                                  .toRotationMatrix();
        const Mat3 covariance = noise_(point.position);
        return {extrinsic_.apply(geometry.sampled[index].point),
                rotation * covariance * rotation.transpose(), point.time, point.id};
    }

    /// 合并有序索引，为更新点与入图点构造观测。
    void materialize(const Geometry& geometry, const SampledIndices& indices,
                     SampledCloud& cloud) const {
        cloud.update.reserve(indices.update.size());
        cloud.merge.reserve(indices.merge.size());
        std::size_t update = 0, merge = 0;
        while (update < indices.update.size() || merge < indices.merge.size()) {
            auto index =
                merge < indices.merge.size() ? indices.merge[merge] : indices.update[update];
            if (update < indices.update.size())
                index = std::min(index, indices.update[update]);
            const auto point = observation(geometry, index);
            if (update < indices.update.size() && indices.update[update] == index) {
                cloud.update.push_back(point);
                ++update;
            }
            if (merge < indices.merge.size() && indices.merge[merge] == index) {
                cloud.merge.push_back(point);
                ++merge;
            }
        }
    }

public:
    /// 选择窗口代表点，并按融合条件物化观测。
    SampledCloud build(WindowPointRange points, const State& reference,
                       const Trajectory& trajectory, const Timing& timing,
                       AdaptiveSampler& sampler) {
        cache_.begin(timing.consumed, reference.time);
        auto prepared_geometry = geometry(points, reference, trajectory, timing);
        const auto indices = sampler.sampleIndices(
            prepared_geometry.sampled, seconds(reference.time - timing.consumed), timing.consumed);
        SampledCloud result;
        materialize(prepared_geometry, indices, result);
        if (output_resolution_ > 0) {
            std::unordered_set<VoxelKey, VoxelHash> cells;
            for (std::size_t i = prepared_geometry.first_new; i < prepared_geometry.sampled.size();
                 ++i) {
                const Vec3 point = extrinsic_.apply(prepared_geometry.sampled[i].point);
                if (cells.insert(voxelKey(point, output_resolution_)).second)
                    result.output.push_back(point);
            }
        }
        return result;
    }

    /// 设置后端点云采样尺度，不影响在线滤波及入图采样。
    void setOutputResolution(double resolution) { output_resolution_ = resolution; }

    /// 提交本次观测后的历史几何缓存。
    void commit(const Pose* correction) { cache_.commit(correction); }

private:
    Pose extrinsic_;
    RayMeasurementCovariance noise_;
    GeometryCache cache_;
    double output_resolution_ = 0;
};

}  // namespace zlio
}  // namespace asuka
