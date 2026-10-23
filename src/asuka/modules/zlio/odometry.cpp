/**
 * @file odometry.cpp
 * @brief 编排启动、惯性传播、激光校正和地图提交。
 */
#include <asuka/modules/zlio/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "preparation.hpp"
#include <asuka/modules/zlio/state.hpp>
namespace asuka {
namespace zlio {
namespace {
/// 将倾角、速度与零偏不确定度搬到新误差图表，为位置和航向建立新规范先验。
Mat15 warmCovariance(const Estimate& previous, const Mat3& rotation, const Vec3& yaw_gradient,
                     const Config& config) {
    Mat15 retained = Mat15::Zero();
    retained.block<3, 3>(3, 3) = rotation - Vec3::UnitZ() * yaw_gradient.transpose();
    retained.block<3, 3>(6, 6) = rotation;
    retained.block<3, 3>(9, 9) = rotation;
    retained.block<3, 3>(12, 12) = rotation;
    Mat15 covariance = retained * previous.covariance * retained.transpose();
    const double gauge_variance = config.initialization.registration.initial_state_variance;
    covariance.topLeftCorner<3, 3>().diagonal().setConstant(gauge_variance);
    covariance(5, 5) += gauge_variance;
    return .5 * (covariance + covariance.transpose()).eval();
}
}  // namespace

/// 构造拥有独立职责的估计组件。
Odometry::Odometry(Config config, InputBufferLimits limits)
    : config_(normalizeConfig(std::move(config))),
      input_(config_, limits),
      initializer_(config_),
      sampler_(config_.sampling),
      observations_(std::make_unique<Preparation>(config_)),
      map_(config_.map),
      linearizer_(config_.association_threads),
      gravity_(0, 0, -config_.gravity) {
}
/// 释放估计会话及其工作区。
Odometry::~Odometry() = default;
/// 校验独立输出采样分辨率。
void Odometry::setOutputResolution(double resolution) {
    if (!std::isfinite(resolution) || resolution < 0 || (resolution > 0 && resolution < 1e-4))
        throw std::invalid_argument("output cloud resolution must be zero or at least 0.1 mm");
    observations_->setOutputResolution(resolution);
    output_resolution_ = resolution;
}
/// 建立新的 odom 规范，保留运动状态并从连续惯性历史恢复处理。
bool Odometry::resetWarm() {
    if (!initialized_ || !input_.imus().covers(filter_.state().time))
        return false;
    const Estimate previous = filter_.estimate();
    State state = previous.state;
    const Mat3 rotation = state.pose.rotation.toRotationMatrix();
    // 绕世界重力轴去掉航向，不改变 IMU 系中的重力方向；X 轴近竖直时使用 Y 轴。
    const bool use_x = rotation.col(0).head<2>().squaredNorm() > 1e-12;
    const Vec3 heading = rotation.col(use_x ? 0 : 1);
    const double yaw =
        use_x ? std::atan2(heading.y(), heading.x()) : std::atan2(-heading.x(), heading.y());
    const Vec3 yaw_gradient(-heading.z() * heading.x() / heading.head<2>().squaredNorm(),
                            -heading.z() * heading.y() / heading.head<2>().squaredNorm(), 1.);
    const Mat3 world_rotation = Eigen::AngleAxisd(-yaw, Vec3::UnitZ()).toRotationMatrix();
    state.pose.rotation =
        (Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Vec3::UnitZ())) * state.pose.rotation)
            .normalized();
    state.pose.position.setZero();
    state.velocity = world_rotation * state.velocity;
    ErrorStateFilter next_filter;
    next_filter.reset(state, warmCovariance(previous, world_rotation, yaw_gradient, config_));
    auto next = std::make_unique<Preparation>(config_);
    next->setOutputResolution(output_resolution_);
    // 先准备和校验替代状态，再清理旧地图；重置时刻仍保留真实 IMU 插值支撑。
    input_.restartTracking(state.time);
    linearizer_.beginUpdate(0);
    map_.clear();
    observations_ = std::move(next);
    trajectory_ = Trajectory();
    sampler_ = AdaptiveSampler(config_.sampling);
    filter_ = std::move(next_filter);
    initialization_time_ = state.time;
    return true;
}
/// 接收惯性测量并尝试推进启动会话。
void Odometry::addImu(const Imu& sample) {
    if (!input_.addImu(sample))
        return;
    if (!initialized_)
        tryInitialize();
}
/// 接收雷达回波并尝试推进启动会话。
void Odometry::addScan(Scan scan) {
    input_.addScan(std::move(scan));
    if (!initialized_)
        tryInitialize();
}
/// 返回已提交后验或启动边界的时间。
std::optional<Time> Odometry::posteriorWatermark() const {
    return initialized_ ? std::optional<Time>(filter_.state().time) : input_.startupBoundary();
}
/// 安装启动会话生成的导航估计与种子地图。
void Odometry::acceptInitialization(const InitializationResult& result, Time end) {
    if (result.map.empty())
        throw std::logic_error("initialization handover requires an accepted seed map");
    filter_.reset(result.estimate.state, result.estimate.covariance);
    gravity_ = result.gravity;
    map_.insert(result.map);
    initialized_ = true;
    initialization_time_ = end;
    input_.start(end);
}
/// 依次处理具有完整惯性覆盖的启动扫描。
void Odometry::tryInitialize() {
    while (auto packet = input_.takeStartup()) {
        const auto boundary = input_.startupBoundary();
        if (!boundary) {
            if (packet->end < input_.imus().samples().front().time)
                continue;
            input_.advanceStartup(packet->end);
            continue;
        }
        if (packet->end > *boundary) {
            const auto bundle = input_.startupImus(packet->end);
            auto& points = packet->points;
            points.erase(
                std::remove_if(points.begin(), points.end(),
                               [&](const Point& point) { return point.time <= *boundary; }),
                points.end());
            if (points.empty())
                continue;
            if (auto result = initializer_.add(points, packet->end, bundle)) {
                acceptInitialization(*result, packet->end);
                return;
            }
            input_.advanceStartup(packet->end);
        } else {
            input_.advanceStartup(*boundary);
        }
    }
}
/// 顺序积分惯性区间并保存对应的名义轨迹。
void Odometry::predictTo(Time time) {
    const auto& history = input_.imus();
    const auto& samples = history.samples();
    Imu left = history.at(filter_.state().time);
    auto next = std::upper_bound(samples.begin(), samples.end(), left.time,
                                 [](Time stamp, const Imu& sample) { return stamp < sample.time; });
    while (left.time < time) {
        const Imu right = next != samples.end() && next->time <= time ? *next++ : history.at(time);
        trajectory_.append(filter_.predict(left, right, gravity_, config_.noise));
        left = right;
    }
}
/// 处理一个观测端点并按融合状态提交后验。
FrameResult Odometry::process(Time end) {
    const Time observation_begin = input_.consumed();
    predictTo(end);
    const Time begin =
        std::max({end - nanoseconds(config_.window.maximum_interval), initialization_time_,
                  input_.imus().minimumTime().value_or(initialization_time_)});
    const Vec6 curvature = trajectory_.curvature(input_.imus(), filter_.state(), begin);
    auto sampled = observations_->build(input_.points(end), filter_.state(), trajectory_,
                                        {initialization_time_, input_.consumed()}, sampler_);
    UpdateResult update;
    Linearization last_observation;
    State last_linearization;
    const Pose predicted_pose = filter_.state().pose;
    // 每个有界观测区间只融合一次，不用点数替代几何信息判断。
    {
        linearizer_.beginUpdate(sampled.update.size());
        update = filter_.update(
            [&](const State& state, const Mat15& covariance) {
                last_linearization = state;
                last_observation = linearizer_.linearize(sampled.update, map_, state, covariance);
                // 观测校正与时间采样使用同一份信息。
                return last_observation;
            },
            config_.maximum_iterations, config_.convergence_tolerance);
    }
    if (update.accepted) {
        Pose correction;
        correction.rotation = filter_.state().pose.rotation * predicted_pose.rotation.conjugate();
        correction.position =
            filter_.state().pose.position - correction.rotation * predicted_pose.position;

        trajectory_.correct(input_.consumed(), correction);
        observations_->commit(&correction);
    } else {
        observations_->commit(nullptr);
    }
    map_.insert(sampled.merge, filter_.estimate());
    double next_interval = config_.window.maximum_interval;
    if (last_observation.count > 0 && update.accepted) {
        const Mat15 chart = priorErrorJacobian(filter_.state(), last_linearization);
        const Mat6 info =
            (chart.transpose() * last_observation.information * chart).topLeftCorner<6, 6>();
        next_interval = observationInterval(filter_.state(), info, last_observation.count,
                                            curvature, config_.window);
    }
    input_.advance(end, next_interval);
    const Time oldest = end - nanoseconds(config_.window.maximum_interval);
    trajectory_.discardBefore(oldest);
    FrameResult result;
    result.estimate = filter_.estimate();
    result.angular_velocity = input_.imus().at(end).angular_velocity;
    result.begin = observation_begin;
    result.cloud = std::move(sampled.output);
    result.matches = last_observation.count;
    result.lidar_updated = update.accepted;
    return result;
}

/// 依次处理已到齐的观测时间片。
std::vector<FrameResult> Odometry::processAvailable(bool flush) {
    std::vector<FrameResult> results;
    if (initialized_)
        while (auto end = input_.nextEnd(flush))
            results.push_back(process(*end));
    return results;
}
}  // namespace zlio
}  // namespace asuka
