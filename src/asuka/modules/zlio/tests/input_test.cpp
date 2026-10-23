/**
 * @file input_test.cpp
 * @brief 验证时间连续性、消费水位、回波身份和输入容量的分类合同。
 */
#include <asuka/modules/zlio/input.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>

#include <asuka/modules/zlio/inertial.hpp>
#include <asuka/modules/zlio/sampling.hpp>

namespace asuka {
namespace zlio {
namespace {
/// 构造指定时间及单轴加速度的惯性采样。
Imu sample(Time time, double acceleration = 0) {
    Imu value;
    value.time = time;
    value.acceleration.x() = acceleration;
    return value;
}

TEST(ImuHistory, InterpolatesAndRetainsOriginalIntervalAtWatermark) {
    ImuHistory history;
    history.push(sample(100, 1));
    history.push(sample(200, 3));
    history.push(sample(300, 5));
    history.discardBefore(150);
    EXPECT_DOUBLE_EQ(history.at(150).acceleration.x(), 2);
    EXPECT_EQ(originalImuInterval(history.at(150), history.at(200)), 100);
    EXPECT_THROW(history.at(149), std::exception);
    EXPECT_THROW(history.at(301), std::exception);
    EXPECT_EQ(history.samples().front().time, 100);
}

/// 非有限测量始终作为无效输入拒绝，包括重复及回退时间戳。
TEST(ImuHistory, NonfiniteAppendDoesNotConsumeHistory) {
    ImuHistory history;
    ASSERT_TRUE(history.push(sample(100, 1)));
    for (const Time time : {99, 100, 200}) {
        EXPECT_THROW(history.push(sample(time, std::numeric_limits<double>::quiet_NaN())),
                     std::invalid_argument);
        auto invalid = sample(time);
        invalid.angular_velocity.z() = std::numeric_limits<double>::infinity();
        EXPECT_THROW(history.push(invalid), std::invalid_argument);
    }
    ASSERT_EQ(history.samples().size(), 1U);
    EXPECT_DOUBLE_EQ(history.at(100).acceleration.x(), 1);
    history.push(sample(200));
    EXPECT_EQ(history.samples().size(), 2U);
}

/// 重复采样不覆盖原始测量，时间回退不改变水位和可插值区间。
TEST(ImuHistory, DuplicatesAreIgnoredAndRewindsPreserveExistingHistory) {
    ImuHistory history;
    ASSERT_TRUE(history.push(sample(100, 1)));
    ASSERT_TRUE(history.push(sample(200, 3)));
    history.discardBefore(150);
    const auto watermark = history.minimumTime();
    EXPECT_FALSE(history.push(sample(200, 100)));
    for (const Time time : {199, 100, 0, -1}) {
        SCOPED_TRACE(time);
        EXPECT_THROW(history.push(sample(time)), InputDiscontinuity);
    }
    EXPECT_EQ(history.minimumTime(), watermark);
    ASSERT_EQ(history.samples().size(), 2U);
    EXPECT_DOUBLE_EQ(history.at(150).acceleration.x(), 2);
    EXPECT_DOUBLE_EQ(history.at(200).acceleration.x(), 3);
    EXPECT_TRUE(history.push(sample(300, 5)));
}

TEST(ImuHistory, CountBudgetAdvancesQueryableWatermark) {
    ImuHistory history({30, 3});
    for (Time time = 1; time <= 6; ++time)
        history.push(sample(time));
    EXPECT_LE(history.samples().size(), 3U);
    EXPECT_THROW(history.at(1), std::exception);
    EXPECT_EQ(history.at(6).time, 6);
}

/// 历史覆盖同时受时长、数量和显式消费水位约束。
TEST(ImuHistory, CoverageUsesBothWatermarkAndLatestSample) {
    ImuHistory history({.15, 3});
    EXPECT_FALSE(history.covers(0));
    for (int index = 0; index <= 3; ++index)
        history.push(sample(nanoseconds(.1 * index)));
    EXPECT_FALSE(history.covers(nanoseconds(.15) - 1));
    EXPECT_TRUE(history.covers(nanoseconds(.15)));
    EXPECT_TRUE(history.covers(nanoseconds(.3)));
    EXPECT_FALSE(history.covers(nanoseconds(.3) + 1));
    history.discardBefore(nanoseconds(.25));
    EXPECT_FALSE(history.covers(nanoseconds(.2)));
    EXPECT_TRUE(history.covers(nanoseconds(.25)));
}

/// 采样间隔上限独立于历史容量，拒绝断流时不改变已有历史。
TEST(ImuHistory, RejectsSamplingGapBeforeChangingHistory) {
    ImuHistory history({30, 65536, .1});
    history.push(sample(nanoseconds(1)));
    history.push(sample(nanoseconds(1.1)));
    const auto watermark = history.minimumTime();
    EXPECT_THROW(history.push(sample(nanoseconds(1.2) + 1)), InputDiscontinuity);
    EXPECT_THROW(history.push(sample(nanoseconds(11.1))), InputDiscontinuity);
    EXPECT_THROW(history.push(sample(nanoseconds(32.1))), InputDiscontinuity);
    EXPECT_EQ(history.samples().size(), 2U);
    EXPECT_EQ(history.minimumTime(), watermark);
    EXPECT_EQ(history.samples().back().time, nanoseconds(1.1));
    EXPECT_NO_THROW(history.push(sample(nanoseconds(1.2))));
}

/// 纳秒边界上的采样连续性判定不得发生整数溢出。
TEST(ImuHistory, GapCheckHandlesTimestampExtremes) {
    ImuHistory history({30, 3, 1e-9});
    const Time minimum = std::numeric_limits<Time>::min();
    history.push(sample(minimum));
    history.push(sample(minimum + 1));
    EXPECT_THROW(history.push(sample(std::numeric_limits<Time>::max())), InputDiscontinuity);
    EXPECT_EQ(history.samples().back().time, minimum + 1);
}

/// 间隔配置必须能够表示为正整数纳秒。
TEST(ImuHistory, RejectsInvalidSamplingIntervals) {
    for (const double interval : {0., -1., 1e-12, 1e20, std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW((ImuHistory{ImuHistoryLimits{30, 65536, interval}}), std::invalid_argument);
}

/// 等待 IMU 的时间覆盖超限属于可恢复中断，单个超大输入包仍按容量错误拒绝。
TEST(Input, MissingImuCoverageIsDistinctFromCapacityFailure) {
    Input delayed(Config{}, {});
    delayed.addImu(sample(nanoseconds(1)));
    EXPECT_THROW(delayed.addScan({{Vec3(1, 0, 0), nanoseconds(32), 0}}), InputDiscontinuity);

    Input waiting(Config{}, {});
    waiting.addScan({{Vec3(1, 0, 0), nanoseconds(1), 0}});
    EXPECT_THROW(waiting.addScan({{Vec3(1, 0, 0), nanoseconds(32), 0}}), InputDiscontinuity);

    InputBufferLimits limits;
    limits.maximum_lidar_points = 1;
    Input bounded(Config{}, limits);
    EXPECT_THROW(bounded.addScan({{Vec3(1, 0, 0), 1, 0}, {Vec3(1, 0, 0), 2, 0}}),
                 std::length_error);
}

/// 启动与运行阶段仅把缺少 IMU 覆盖的累计积压分类为输入中断。
TEST(Input, AccumulatedCapacityFailureDistinguishesMissingImuFromCoveredInput) {
    InputBufferLimits limits;
    limits.maximum_lidar_points = 100;
    for (const bool tracking : {false, true}) {
        for (const bool covered : {false, true}) {
            SCOPED_TRACE(tracking);
            SCOPED_TRACE(covered);
            Input input(Config{}, limits);
            input.addImu(sample(nanoseconds(1)));
            if (covered)
                for (int step = 1; step <= 6; ++step)
                    input.addImu(sample(nanoseconds(1) + step * nanoseconds(.1)));
            if (tracking)
                input.start(nanoseconds(1));
            for (int step = 1; step <= 5; ++step)
                input.addScan(
                    Scan(20, Point{Vec3(1, 0, 0), nanoseconds(1) + step * nanoseconds(.1), 0}));
            const Scan overflow(20, Point{Vec3(1, 0, 0), nanoseconds(1.6), 0});
            if (covered)
                EXPECT_THROW(input.addScan(overflow), std::length_error);
            else
                EXPECT_THROW(input.addScan(overflow), InputDiscontinuity);
        }
    }
    // 首个 IMU 尚未到达时，累计容量用尽同样结束当前等待窗口。
    Input waiting(Config{}, limits);
    waiting.addScan(Scan(100, Point{Vec3(1, 0, 0), nanoseconds(1), 0}));
    EXPECT_THROW(waiting.addScan({{Vec3(1, 0, 0), nanoseconds(1.1), 0}}), InputDiscontinuity);
}

/// 输入编排层保留历史的接收结果，不把重复或回退采样伪装成新输入。
TEST(Input, ImuAdmissionPreservesDuplicateAndRewindClassification) {
    Input input(Config{}, {});
    EXPECT_TRUE(input.addImu(sample(100, 1)));
    EXPECT_FALSE(input.addImu(sample(100, 100)));
    EXPECT_THROW(input.addImu(sample(99)), InputDiscontinuity);
    EXPECT_THROW(input.addImu(sample(100, std::numeric_limits<double>::quiet_NaN())),
                 std::invalid_argument);
    EXPECT_EQ(input.imus().samples().size(), 1U);
    EXPECT_DOUBLE_EQ(input.imus().at(100).acceleration.x(), 1);
    EXPECT_TRUE(input.addImu(sample(200, 3)));
}

TEST(ReturnBuffer, StableMergePreservesDistinctEchoesAndHalfOpenWindow) {
    WindowConfig config;
    config.maximum_interval = 1;
    ReturnBuffer buffer(config);
    buffer.start(0);
    buffer.push({{Vec3(1, 0, 0), 100, 0}, {Vec3(2, 0, 0), 200, 0}});
    buffer.push({{Vec3(3, 0, 0), 150, 0}, {Vec3(4, 0, 0), 200, 0}});
    const auto points = buffer.pointRange(200);
    ASSERT_EQ(points.size(), 4U);
    EXPECT_EQ(points[0].point.time, 100);
    EXPECT_EQ(points[1].point.time, 150);
    EXPECT_DOUBLE_EQ(points[2].point.position.x(), 2);
    EXPECT_DOUBLE_EQ(points[3].point.position.x(), 4);
    std::set<std::uint64_t> ids;
    for (const auto& point : points)
        ids.insert(point.point.id);
    EXPECT_EQ(ids.size(), points.size());
    EXPECT_EQ(buffer.pointRange(nanoseconds(1) + 100).size(), 3U);
}

TEST(AdaptiveSampling, ReturnsOrderedOriginalIndicesStrictlyAfterWatermark) {
    AdaptiveSampler sampler;
    std::vector<SamplingPoint> points;
    for (int index = 0; index < 30; ++index)
        points.push_back({Vec3(.5 * index, 0, 1), index});
    const auto result = sampler.sampleIndices(points, 0, 10);
    for (const auto* indices : {&result.update, &result.merge}) {
        ASSERT_FALSE(indices->empty());
        EXPECT_TRUE(std::is_sorted(indices->begin(), indices->end()));
        std::set<std::size_t> unique(indices->begin(), indices->end());
        EXPECT_EQ(unique.size(), indices->size());
        for (auto index : *indices) {
            ASSERT_LT(index, points.size());
            EXPECT_GT(points[index].time, 10);
        }
    }
}
}  // namespace
}  // namespace zlio
}  // namespace asuka
