/**
 * @file prediction.cpp
 * @brief 从激光后验重放惯性测量并生成高频预测。
 */
#include <asuka/odometry/zlio/prediction.hpp>
namespace asuka {
namespace zlio {
/// 加入有序 IMU 样本，并在已接受后验后推进高频预测。
std::optional<Estimate> OutputPredictor::addImu(const Imu& sample) {
    if (!imus_.push(sample) || !initialized_ || sample.time <= predictor_.state().time)
        return std::nullopt;
    if (!imus_.covers(predictor_.state().time))
        throw InputDiscontinuity("prediction state precedes the retained IMU history watermark");
    predictor_.predict(imus_.at(predictor_.state().time), sample, gravity_, noise_);
    return predictor_.estimate();
}

/// 接受未被覆盖的后验，重放其后的 IMU 并清退不再需要的历史。
bool OutputPredictor::correct(const Estimate& posterior, std::optional<Vec3> gravity) {
    if (initialized_ && posterior.state.time <= posterior_time_)
        return false;
    reset(posterior, gravity);
    return true;
}

/// 构造新的预测状态，重传播成功后一起替换后验水位、重力与预测结果。
void OutputPredictor::reset(const Estimate& posterior, std::optional<Vec3> gravity) {
    if (const auto minimum = imus_.minimumTime(); minimum && posterior.state.time < *minimum)
        throw InputDiscontinuity("posterior state precedes the retained IMU history watermark");
    Imu left = imus_.at(posterior.state.time);
    const Vec3 next_gravity = gravity.value_or(gravity_);
    ErrorStateFilter next;
    next.reset(posterior.state, posterior.covariance);
    for (const auto& sample : imus_.samples()) {
        if (sample.time <= left.time)
            continue;
        const Imu right = sample;
        next.predict(left, right, next_gravity, noise_);
        left = right;
    }
    predictor_ = std::move(next);
    gravity_ = next_gravity;
    posterior_time_ = posterior.state.time;
    initialized_ = true;
    imus_.discardBefore(posterior_time_);
}

}  // namespace zlio
}  // namespace asuka
