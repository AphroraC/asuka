/**
 * @file initialization_test.cpp
 * @brief 验证重力一致的启动、持续建图和暖重置行为。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <asuka/modules/zlio/odometry.hpp>
#include <asuka/modules/zlio/state.hpp>

namespace asuka {
namespace zlio {
namespace {
/// 构造三个正交平面的定时扫描，可选单精度坐标及孤立回波。
Scan roomScan(Time begin, Time end, bool float_coordinates = false, bool isolated_return = false) {
    Scan scan;
    for (int first = -20; first <= 20; ++first)
        for (int second = -20; second <= 20; ++second) {
            const double u = float_coordinates ? static_cast<float>(.1 * first) : .1 * first;
            const double v = float_coordinates ? static_cast<float>(.1 * second) : .1 * second;
            scan.push_back({Vec3(4, u, v), 0, 0});
            scan.push_back({Vec3(u, 4, v), 0, 0});
            scan.push_back({Vec3(u, v, -2), 0, 0});
        }
    for (std::size_t index = 0; index < scan.size(); ++index)
        scan[index].time =
            begin + (end - begin) * static_cast<Time>(index + 1) / static_cast<Time>(scan.size());
    if (isolated_return)
        scan.push_back({Vec3(4.5, 0, 0), end, 0});
    return scan;
}

class StationaryMapping : public ::testing::TestWithParam<std::tuple<bool, bool>> {};

/// 连续静止观测在完成初始化后保持有限后验及有效雷达更新。
TEST_P(StationaryMapping, RepeatedGeometryKeepsFinitePosteriorsAfterInitialization) {
    const auto [float_coordinates, isolated_return] = GetParam();
    const Config config;  // 默认值 == config_zlio.json（迁移期核对一致）
    Odometry odometry(config);
    constexpr Time kStartTime = 1000000000LL;
    constexpr Time kImuPeriod = 10000000LL;
    constexpr Time kScanPeriod = 100000000LL;
    std::vector<FrameResult> outputs;
    for (int step = 1; step <= 350; ++step) {
        Imu imu;
        imu.time = kStartTime + step * kImuPeriod;
        imu.acceleration.z() = config.gravity;
        odometry.addImu(imu);
        if (step % 10 == 0) {
            odometry.addScan(
                roomScan(imu.time - kScanPeriod, imu.time, float_coordinates, isolated_return));
            const auto batch = odometry.processAvailable();
            outputs.insert(outputs.end(), batch.begin(), batch.end());
        }
    }
    const auto tail = odometry.processAvailable(true);
    outputs.insert(outputs.end(), tail.begin(), tail.end());
    ASSERT_TRUE(odometry.initialized()) << odometry.initializationStatus().reason;
    ASSERT_GE(outputs.size(), 10U);
    for (const auto& frame : outputs) {
        EXPECT_TRUE(frame.lidar_updated);
        EXPECT_GT(frame.matches, 0U);
        EXPECT_LT(frame.estimate.state.velocity.norm(), .02);
        EXPECT_LT(frame.estimate.state.pose.position.norm(), .001);
        EXPECT_TRUE(numericallyPositiveCovariance(frame.estimate.covariance));
    }
}

/// 为坐标精度与孤立回波组合生成明确的用例名称。
std::string stationaryScenarioName(
    const ::testing::TestParamInfo<StationaryMapping::ParamType>& info) {
    const auto [float_coordinates, isolated_return] = info.param;
    return std::string(float_coordinates ? "FloatCoordinates" : "DoubleCoordinates") +
           (isolated_return ? "WithIsolatedReturn" : "WithoutIsolatedReturn");
}

INSTANTIATE_TEST_SUITE_P(CoordinatesAndSupport, StationaryMapping,
                         ::testing::Combine(::testing::Bool(), ::testing::Bool()),
                         stationaryScenarioName);

/// 完成启动后执行暖重置，验证新局部地图、偏置保留及待处理数据清空。
TEST(OdometryInitialization, SupportedRoomHandsOffOnceAndWarmResetStartsNewLocalMap) {
    Config config;
    config.association_threads = 1;
    config.initialization.optimization.thread_count = 1;
    Odometry odometry(config);
    std::vector<FrameResult> outputs;
    for (int step = 1; step <= 500; ++step) {
        Imu imu;
        imu.time = nanoseconds(1 + .01 * step);
        imu.acceleration.z() = config.gravity;
        odometry.addImu(imu);
        if (step % 10 == 0) {
            odometry.addScan(roomScan(imu.time - nanoseconds(.1), imu.time));
            const auto batch = odometry.processAvailable();
            outputs.insert(outputs.end(), batch.begin(), batch.end());
        }
        if (outputs.size() >= 3)
            break;
    }
    ASSERT_TRUE(odometry.initialized()) << odometry.initializationStatus().reason;
    ASSERT_GE(outputs.size(), 3U);
    EXPECT_LT(odometry.estimate().state.velocity.norm(), 1e-5);
    EXPECT_TRUE(numericallyPositiveCovariance(odometry.estimate().covariance));
    for (std::size_t index = 1; index < outputs.size(); ++index)
        EXPECT_GT(outputs[index].estimate.state.time, outputs[index - 1].estimate.state.time);
    const auto bias = odometry.estimate().state.accel_bias;
    const auto speed = odometry.estimate().state.velocity.norm();
    ASSERT_TRUE(odometry.resetWarm());
    EXPECT_LT(odometry.estimate().state.pose.position.norm(), 1e-12);
    EXPECT_NEAR(odometry.estimate().state.velocity.norm(), speed, 1e-12);
    EXPECT_LT((odometry.estimate().state.accel_bias - bias).norm(), 1e-12);
    EXPECT_TRUE(odometry.initialized());
    EXPECT_TRUE(odometry.processAvailable(true).empty());
}

/// 以物理状态直接定义新局部坐标系，供数值差分独立计算误差图表变换。
State stateInResetFrame(State state) {
    const Mat3 rotation = state.pose.rotation.toRotationMatrix();
    const Vec3 heading = rotation.col(0);
    const Eigen::Quaterniond new_from_old(
        Eigen::AngleAxisd(-std::atan2(heading.y(), heading.x()), Vec3::UnitZ()));
    state.pose.rotation = (new_from_old * state.pose.rotation).normalized();
    state.pose.position.setZero();
    state.velocity = new_from_old * state.velocity;
    return state;
}

/// 运动中重置保留机体系速度，并将完整协方差按数值差分得到的 Jacobian 搬到新图表。
TEST(OdometryInitialization, MovingWarmResetPreservesVelocityAndTransformsCovariance) {
    Config config;
    config.association_threads = 1;
    config.initialization.optimization.thread_count = 1;
    Odometry odometry(config);
    const Time motion_begin = nanoseconds(4.5);
    const Vec3 acceleration(.4, -.2, .1);
    const Vec3 angular_velocity(.05, -.04, .3);
    for (int step = 1; step <= 550; ++step) {
        Imu imu;
        imu.time = nanoseconds(1) + step * nanoseconds(.01);
        const double duration = std::max(0., seconds(imu.time - motion_begin));
        const auto rotation = expRotation(duration * angular_velocity);
        imu.acceleration = duration > 0
                               ? rotation.conjugate() * (acceleration + Vec3(0, 0, config.gravity))
                               : Vec3(0, 0, config.gravity);
        imu.angular_velocity = duration > 0 ? angular_velocity : Vec3::Zero();
        odometry.addImu(imu);
        if (step % 10 == 0) {
            auto scan = roomScan(imu.time - nanoseconds(.1), imu.time);
            for (auto& point : scan) {
                const double elapsed = std::max(0., seconds(point.time - motion_begin));
                point.position = expRotation(elapsed * angular_velocity).conjugate() *
                                 (point.position - .5 * elapsed * elapsed * acceleration);
            }
            odometry.addScan(std::move(scan));
            odometry.processAvailable();
        }
    }
    ASSERT_TRUE(odometry.initialized()) << odometry.initializationStatus().reason;
    const Estimate previous = odometry.estimate();
    ASSERT_GT(previous.state.velocity.norm(), .5);
    const State expected = stateInResetFrame(previous.state);
    Mat15 jacobian;
    constexpr double kStep = 1e-6;
    for (int column = 0; column < 15; ++column) {
        const Vec15 delta = kStep * Vec15::Unit(column);
        jacobian.col(column) =
            (difference(stateInResetFrame(retract(previous.state, delta)), expected) -
             difference(stateInResetFrame(retract(previous.state, -delta)), expected)) /
            (2 * kStep);
    }
    Mat15 expected_covariance = jacobian * previous.covariance * jacobian.transpose();
    const double gauge_variance = config.initialization.registration.initial_state_variance;
    expected_covariance.topLeftCorner<3, 3>().diagonal().array() += gauge_variance;
    expected_covariance(5, 5) += gauge_variance;
    ASSERT_TRUE(odometry.resetWarm());
    const Estimate reset = odometry.estimate();
    EXPECT_LT(difference(reset.state, expected).norm(), 1e-12);
    EXPECT_EQ(reset.state.time, previous.state.time);
    EXPECT_LT((reset.state.pose.rotation.conjugate() * reset.state.velocity -
               previous.state.pose.rotation.conjugate() * previous.state.velocity)
                  .norm(),
              1e-12);
    EXPECT_LT((reset.covariance - expected_covariance).norm(), 1e-7 * expected_covariance.norm());
    EXPECT_TRUE(numericallyPositiveCovariance(reset.covariance));
}

/// 雷达长时间中断后，暖重置明确拒绝失去历史支撑的旧后验且不改变状态。
TEST(OdometryInitialization, ExpiredHistoryDeclinesWarmResetWithoutMutation) {
    Config config;
    config.association_threads = 1;
    config.initialization.optimization.thread_count = 1;
    Odometry odometry(config);
    Imu imu;
    imu.acceleration.z() = config.gravity;
    for (int step = 1; step <= 350; ++step) {
        imu.time = nanoseconds(1) + step * nanoseconds(.01);
        odometry.addImu(imu);
        if (step % 10 == 0) {
            odometry.addScan(roomScan(imu.time - nanoseconds(.1), imu.time));
            odometry.processAvailable();
        }
    }
    ASSERT_TRUE(odometry.initialized()) << odometry.initializationStatus().reason;
    const Estimate previous = odometry.estimate();
    bool expired = false;
    for (int step = 0; step < 3100; ++step) {
        imu.time += nanoseconds(.01);
        try {
            odometry.addImu(imu);
        } catch (const std::runtime_error&) {
            expired = true;
            break;
        }
    }
    ASSERT_TRUE(expired);
    for (int attempt = 0; attempt < 2; ++attempt) {
        EXPECT_FALSE(odometry.resetWarm());
        EXPECT_EQ(odometry.estimate().state.time, previous.state.time);
        EXPECT_EQ(difference(odometry.estimate().state, previous.state).norm(), 0);
        EXPECT_EQ((odometry.estimate().covariance - previous.covariance).norm(), 0);
    }
}
}  // namespace
}  // namespace zlio
}  // namespace asuka
