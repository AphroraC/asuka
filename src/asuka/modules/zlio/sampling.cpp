/**
 * @file sampling.cpp
 * @brief 根据运动曲率选择观测时长，并按空间密度选择代表回波。
 */
#include <asuka/modules/zlio/sampling.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <asuka/modules/zlio/state.hpp>
#include <asuka/modules/zlio/voxel.hpp>

namespace asuka {
namespace zlio {

/// 按当前观测信息度量下的二阶运动外推缺陷选择下一观测时长。
double observationInterval(const State& state, const Mat6& information, std::size_t matches,
                           const Vec6& curvature, const WindowConfig& config) {
    if (matches == 0)
        return config.maximum_interval;
    Mat6 transform = Mat6::Zero();
    const Mat3 rotation = state.pose.rotation.toRotationMatrix();
    transform.topLeftCorner<3, 3>() = rotation;
    transform.bottomRightCorner<3, 3>() = rotation;
    transform.topRightCorner<3, 3>() = skew(state.pose.position) * rotation;
    const Mat6 metric =
        transform.transpose() * information * transform / static_cast<double>(matches);
    const double coefficient = std::max(0., .25 * curvature.dot(metric * curvature));
    if (coefficient == 0.)
        return config.maximum_interval;
    const double budget = config.normalized_error * config.normalized_error;
    return std::clamp(std::pow(budget / coefficient, .25), config.minimum_interval,
                      config.maximum_interval);
}

namespace {
/// 索引连续采样格的体素键，键和数据由调用方共同拥有，不保存借用引用。
class VoxelIndex {
public:
    /// 清除本批短指纹，保留桶数组以供下一窗口复用。
    void clear() { std::fill(fingerprints_.begin(), fingerprints_.end(), 0); }

    /// 保持桶占用率不超过四分之三，扩容时依据已有格键重建桶位置。
    template <class Records>
    void reserve(std::size_t count, const Records& records) {
        std::size_t capacity = std::max<std::size_t>(buckets_.size(), 16);
        while (count > capacity - capacity / 4) {
            if (capacity > buckets_.max_size() / 2)
                throw std::length_error("sampling voxel index capacity exceeded");
            capacity *= 2;
        }
        if (capacity == buckets_.size())
            return;
        std::vector<std::size_t> buckets(capacity);
        std::vector<std::uint8_t> fingerprints(capacity, 0);
        for (std::size_t index = 0; index < records.size(); ++index) {
            const auto hash_value = hash(records[index].key);
            std::size_t bucket = hash_value & (capacity - 1);
            while (fingerprints[bucket] != 0)
                bucket = (bucket + 1) & (capacity - 1);
            buckets[bucket] = index;
            fingerprints[bucket] = fingerprint(hash_value);
        }
        buckets_.swap(buckets);
        fingerprints_.swap(fingerprints);
    }

    /// 返回已有格槽号或数组末端；新键由调用方紧接着追加为完整格记录。
    template <class Records>
    std::size_t slot(const VoxelKey& key, const Records& records) {
        if (buckets_.empty())
            reserve(1, records);
        const auto hash_value = hash(key);
        std::size_t bucket = locate(key, hash_value, records);
        if (fingerprints_[bucket] != 0)
            return buckets_[bucket];
        if (records.size() >= buckets_.size() - buckets_.size() / 4) {
            reserve(records.size() + 1, records);
            bucket = locate(key, hash_value, records);
        }
        buckets_[bucket] = records.size();
        fingerprints_[bucket] = fingerprint(hash_value);
        return records.size();
    }

    /// 返回桶数量，供三张采样表执行共同缩容规则。
    std::size_t capacity() const { return buckets_.size(); }

private:
    /// 混合体素哈希的高低位，减少规则坐标对二次幂桶数的碰撞。
    static std::uint64_t hash(const VoxelKey& key) {
        std::uint64_t value = VoxelHash{}(key);
        value ^= value >> 30;
        value *= 0xbf58476d1ce4e5b9ULL;
        value ^= value >> 27;
        value *= 0x94d049bb133111ebULL;
        return value ^ (value >> 31);
    }

    /// 提取非零短指纹，零值专用于标记空桶。
    static std::uint8_t fingerprint(std::uint64_t hash_value) {
        return static_cast<std::uint8_t>((hash_value >> 56) | 1);
    }

    /// 线性探测桶，只有短指纹相同时才读取连续数组中的完整键。
    template <class Records>
    std::size_t locate(const VoxelKey& key, std::uint64_t hash_value,
                       const Records& records) const {
        const std::size_t mask = buckets_.size() - 1;
        const auto expected = fingerprint(hash_value);
        std::size_t bucket = hash_value & mask;
        while (fingerprints_[bucket] != 0) {
            if (fingerprints_[bucket] == expected && records[buckets_[bucket]].key == key)
                break;
            bucket = (bucket + 1) & mask;
        }
        return bucket;
    }

    std::vector<std::size_t> buckets_;
    std::vector<std::uint8_t> fingerprints_;
};

/// 保存离格中心最近的原始回波，等距时选择输入索引较小者。
struct Representative {
    std::size_t index = std::numeric_limits<std::size_t>::max();
    double distance = std::numeric_limits<double>::infinity();

    /// 根据中心距离与稳定索引规则更新代表回波。
    void consider(std::size_t candidate, double candidate_distance) {
        if (!valid() || candidate_distance < distance ||
            (candidate_distance == distance && candidate < index)) {
            index = candidate;
            distance = candidate_distance;
        }
    }
    /// 判断该格是否已经接纳代表回波。
    bool valid() const { return index != std::numeric_limits<std::size_t>::max(); }
};

/// 配对采样专用索引与连续格数据；不暴露删除或通用映射操作。
template <class Cell>
struct SamplingCells {
    struct Record {
        VoxelKey key;
        Cell value;
    };
    VoxelIndex index;
    std::vector<Record> values;

    /// 清空本批数据并为全部候选预留容量。
    void prepare(std::size_t count) {
        index.clear();
        values.clear();
        index.reserve(count, values);
        values.reserve(count);
    }

    /// 获取体素对应的数据，新体素仅追加一个默认格。
    Cell& at(const VoxelKey& key) {
        const std::size_t slot = index.slot(key, values);
        if (slot == values.size())
            values.push_back({key, {}});
        return values[slot].value;
    }
};
using Cells = SamplingCells<Representative>;

/// 在指定体素尺寸下选取全窗口代表回波。
void representatives(const std::vector<SamplingPoint>& input, double size, Cells& cells) {
    if (!(size > 0) || !std::isfinite(size))
        throw std::invalid_argument("voxel size must be finite and positive");
    cells.prepare(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        const VoxelKey key = voxelKey(input[i].point, size);
        const double distance = (input[i].point - voxelCenter(key, size)).squaredNorm();
        cells.at(key).consider(i, distance);
    }
}

/// 提取代表回波的距离，供当前帧密度控制器使用。
std::vector<double> representativeRanges(const std::vector<SamplingPoint>& input, double size,
                                         Cells& cells) {
    representatives(input, size, cells);
    std::vector<double> ranges;
    ranges.reserve(cells.values.size());
    for (const auto& cell : cells.values)
        ranges.push_back(input[cell.value.index].point.norm());
    return ranges;
}

/// 执行半尺寸与完整尺寸两级采样，保留新点筛选和浮点父格计算。
SampledIndices sampleWindow(const std::vector<SamplingPoint>& input, double size, Time after,
                            Cells& half, Cells& full) {
    const auto fresh_count = std::count_if(
        input.begin(), input.end(), [after](const auto& point) { return point.time > after; });
    half.prepare(fresh_count);
    const double half_size = size / 2;
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (input[i].time <= after)
            continue;
        const VoxelKey key = voxelKey(input[i].point, half_size);
        const double distance = (input[i].point - voxelCenter(key, half_size)).squaredNorm();
        half.at(key).consider(i, distance);
    }
    full.prepare(half.values.size());
    SampledIndices result;
    result.merge.reserve(half.values.size());
    for (const auto& entry : half.values) {
        const auto fresh = entry.value.index;
        result.merge.push_back(fresh);
        // 独立计算浮点体素键；不假定边界舍入等价于整数父格映射。
        const VoxelKey key = voxelKey(input[fresh].point, size);
        const double distance = (input[fresh].point - voxelCenter(key, size)).squaredNorm();
        full.at(key).consider(fresh, distance);
    }
    result.update.reserve(full.values.size());
    for (const auto& entry : full.values) {
        result.update.push_back(entry.value.index);
    }
    // 体素首次出现顺序不同于最终代表点顺序，两个输出流统一按原输入索引排序。
    std::sort(result.merge.begin(), result.merge.end());
    std::sort(result.update.begin(), result.update.end());
    return result;
}

}  // namespace

/// 独占反馈、半尺寸及完整尺寸三个采样表，并共同执行容量回收规则。
struct AdaptiveSampler::Workspace {
    Cells feedback, half, full;
    /// 小窗口到来时共同释放三张表中过度保留的历史容量。
    void prepare(std::size_t count) {
        const auto capacity =
            std::max({feedback.index.capacity(), half.index.capacity(), full.index.capacity()});
        if (capacity > 4 * std::max<std::size_t>(count, 1024)) {
            feedback = Cells{};
            half = Cells{};
            full = Cells{};
        }
    }
};

/// 按配置建立独立的采样控制器，工作区按需创建。
AdaptiveSampler::AdaptiveSampler(SamplingConfig config)
    : config_(config), size_(config.initial_size) {
}

/// 释放控制器历史与独占采样工作区。
AdaptiveSampler::~AdaptiveSampler() = default;
/// 转移控制器状态及独占工作区。
AdaptiveSampler::AdaptiveSampler(AdaptiveSampler&&) = default;
/// 接管另一采样器的状态与工作区，替换当前内容。
AdaptiveSampler& AdaptiveSampler::operator=(AdaptiveSampler&&) = default;
/// 由距离中位数和点数误差更新体素尺寸，interval 单位为秒。
void AdaptiveSampler::updateSize(std::vector<double> ranges, double interval) {
    auto middle = ranges.begin() + ranges.size() / 2;
    std::nth_element(ranges.begin(), middle, ranges.end());
    double median = *middle;
    if (ranges.size() % 2 == 0)
        median = .5 * (median + *std::max_element(ranges.begin(), middle));
    medians_.push_back(median);
    if (medians_.size() > config_.history_size)
        medians_.pop_front();
    const double scale = std::accumulate(medians_.begin(), medians_.end(), 0.) / medians_.size();
    const double distance_ratio = std::min(scale / config_.scale_threshold, 1.);
    const double density_ratio = 1 - std::pow(1 - distance_ratio, config_.exponent);
    const double target_points =
        config_.minimum_points + (config_.maximum_points - config_.minimum_points) * density_ratio;
    const double error = target_points - static_cast<double>(ranges.size());

    if (first_) {
        previous_error_ = error;
        first_ = false;
    }
    const double derivative = (error - previous_error_) / interval;
    const double proportional_ratio =
        std::min(std::abs(error) / (config_.proportional_scale * target_points), 1.);
    const double derivative_ratio =
        std::min(std::abs(derivative) * interval / (config_.derivative_scale * target_points), 1.);
    const double proportional_gain = config_.proportional_minimum +
                                     (config_.proportional_maximum - config_.proportional_minimum) *
                                         std::sqrt(distance_ratio * proportional_ratio);
    const double derivative_gain =
        config_.derivative_minimum + (config_.derivative_maximum - config_.derivative_minimum) *
                                         std::sqrt(distance_ratio * derivative_ratio);
    size_ = std::clamp(size_ - proportional_gain * error - derivative_gain * derivative,
                       config_.minimum_size, config_.maximum_size);
    previous_error_ = error;
}

/// 选择两个稳定索引流；零时间间隔不推进控制器，超大输入使用临时工作区。
SampledIndices AdaptiveSampler::sampleIndices(const std::vector<SamplingPoint>& input,
                                              double interval, Time after) {
    if (input.empty()) {
        workspace_.reset();
        return {};
    }
    if (!std::isfinite(interval) || interval < 0)
        throw std::invalid_argument("sampling interval duration must be finite and non-negative");
    // 常规窗口复用容量；超大输入不扩大常驻工作区，小输入会释放过大的旧容量。
    constexpr std::size_t kMaximumReusablePoints = 65536;
    std::unique_ptr<Workspace> transient;
    Workspace* workspace;
    if (input.size() > kMaximumReusablePoints) {
        transient = std::make_unique<Workspace>();
        workspace = transient.get();
    } else {
        if (!workspace_)
            workspace_ = std::make_unique<Workspace>();
        workspace = workspace_.get();
    }
    workspace->prepare(input.size());
    // 正时长窗口更新密度反馈，随后执行两级空间采样。
    if (interval > 0)
        updateSize(representativeRanges(input, size_, workspace->feedback), interval);
    return sampleWindow(input, size_, after, workspace->half, workspace->full);
}

}  // namespace zlio
}  // namespace asuka
