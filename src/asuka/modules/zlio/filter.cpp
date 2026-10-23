/**
 * @file filter.cpp
 * @brief 融合观测信息并更新完整导航后验。
 */
#include <asuka/modules/zlio/filter.hpp>

#include <Eigen/Cholesky>
#include <cmath>
#include <optional>
#include <stdexcept>

#include <asuka/modules/zlio/state.hpp>

namespace asuka {
namespace zlio {

namespace {
/// 白化完整先验并求解六维位姿观测，保留全部状态相关项。
class PoseObservationSystem {
public:
    /// 检查观测块结构并建立平方根系统，不适用时交由通用求解路径处理。
    PoseObservationSystem(const Linearization& observation, const Mat15& covariance) {
        if (!observation.information.bottomRows<9>().isZero(0) ||
            !observation.information.rightCols<9>().isZero(0) ||
            !observation.gradient.tail<9>().isZero(0))
            return;
        const Eigen::LLT<Mat15> prior(covariance);
        if (prior.info() != Eigen::Success)
            return;
        root_ = prior.matrixL();
        const Mat6 pose_root = root_.topLeftCorner<6, 6>();
        const Mat6 system = Mat6::Identity() + pose_root.transpose() *
                                                   observation.information.topLeftCorner<6, 6>() *
                                                   pose_root;
        if (!system.allFinite())
            return;
        factor_.compute(system);
        valid_ = factor_.info() == Eigen::Success;
    }
    /// 判断位姿专用系统是否可以安全用于本轮求解。
    bool valid() const { return valid_; }
    /// 在给定先验局部均值附近回代完整十五维增量。
    Vec15 increment(const Linearization& observation, const Vec15& mean) const {
        return mean -
               root_.leftCols<6>() *
                   factor_.solve(root_.topLeftCorner<6, 6>().transpose() *
                                 (observation.gradient.head<6>() +
                                  observation.information.topLeftCorner<6, 6>() * mean.head<6>()));
    }
    /// 更新先验平方根的前六列，再经完整误差重置形成后验协方差。
    Mat15 covariance(const Mat15& reset) const {
        Mat15 updated_root = root_;
        const Eigen::Matrix<double, 6, 15> whitened =
            factor_.matrixL().solve(root_.leftCols<6>().transpose());
        updated_root.leftCols<6>() = whitened.transpose();
        const Mat15 transformed_root = reset * updated_root;
        return transformed_root * transformed_root.transpose();
    }

private:
    Mat15 root_;
    Eigen::LLT<Mat6> factor_;
    bool valid_ = false;
};
}  // namespace

/// 检查初始协方差后，一次性替换完整导航估计。
void ErrorStateFilter::reset(const State& state, const Mat15& covariance) {
    if (!numericallyPositiveCovariance(covariance))
        throw std::invalid_argument(
            "initial covariance failed the numerical positive-definiteness check");
    estimate_ = {state, covariance};
}

/// 推进完整导航估计并生成轨迹段。
ImuSegment ErrorStateFilter::predict(const Imu& left, const Imu& right, const Vec3& gravity,
                                     const ImuNoise& noise) {
    return propagateInertial(estimate_, left, right, gravity, noise);
}

namespace {
/// 以真实状态变化判断收敛，避免导航图表的位置/旋转耦合依赖世界原点。
bool smallPhysicalStep(const State& before, const State& after, const Vec15& navigation_delta,
                       double tolerance) {
    const Vec3 rotation = before.pose.rotation.conjugate() * navigation_delta.segment<3>(3);
    return (after.pose.position - before.pose.position).cwiseAbs().maxCoeff() <= tolerance &&
           rotation.cwiseAbs().maxCoeff() <= tolerance &&
           (after.velocity - before.velocity).cwiseAbs().maxCoeff() <= tolerance &&
           (after.gyro_bias - before.gyro_bias).cwiseAbs().maxCoeff() <= tolerance &&
           (after.accel_bias - before.accel_bias).cwiseAbs().maxCoeff() <= tolerance;
}
}  // namespace

/// 保持传播先验不变完成观测迭代，末轮才提交均值和重置后的协方差。
UpdateResult ErrorStateFilter::update(
    const std::function<Linearization(const State&, const Mat15&)>& linearize,
    int maximum_iterations, double tolerance) {
    if (maximum_iterations < 1 || !std::isfinite(tolerance) || tolerance <= 0)
        throw std::invalid_argument(
            "filter iteration limit and convergence tolerances must be positive");
    const Estimate prior = estimate_;
    std::optional<Mat15> prior_information;
    State iterate = prior.state;
    int converged = 0;
    for (int iteration = 0;; ++iteration) {
        // 先验始终保留在传播时的坐标；每轮通过当前图表 Jacobian 参与求解。
        const Mat15 prior_jacobian = priorErrorJacobian(iterate, prior.state);
        const Vec15 prior_error = difference(iterate, prior.state);
        const Mat15 inverse_jacobian = errorResetJacobian(prior_error);
        const Mat15 chart_covariance =
            inverse_jacobian * prior.covariance * inverse_jacobian.transpose();
        Linearization observation = linearize(iterate, chart_covariance);
        if (observation.count == 0) {
            estimate_ = prior;
            return {};
        }
        Eigen::LDLT<Mat15> factor;
        const PoseObservationSystem pose_system(observation, chart_covariance);
        Vec15 delta;
        if (pose_system.valid()) {
            delta = pose_system.increment(observation, -inverse_jacobian * prior_error);
        } else {
            // 通用十五维观测与数值回退仍使用原信息形式。
            if (!prior_information)
                prior_information = prior.covariance.ldlt().solve(Mat15::Identity());
            const Mat15 information =
                prior_jacobian.transpose() * *prior_information * prior_jacobian +
                observation.information;
            factor.compute(information);
            if (!factor.isPositive())
                throw std::runtime_error(
                    "observation information matrix failed the positive-definiteness check");
            delta = factor.solve(-observation.gradient -
                                 prior_jacobian.transpose() * *prior_information * prior_error);
        }
        if (!delta.allFinite())
            throw std::runtime_error("state update increment contains non-finite values");
        const State next = retract(iterate, delta);
        const bool small_step = smallPhysicalStep(iterate, next, delta, tolerance);
        iterate = next;
        if (small_step)
            ++converged;
        if (converged >= 2 || iteration + 1 == maximum_iterations) {
            const Mat15 reset = errorResetJacobian(delta);
            Mat15 posterior;
            if (pose_system.valid()) {
                posterior = pose_system.covariance(reset);
            } else {
                posterior = reset * factor.solve(Mat15::Identity()) * reset.transpose();
            }
            estimate_ = {iterate, .5 * (posterior + posterior.transpose()).eval()};
            return {true};
        }
    }
}

}  // namespace zlio
}  // namespace asuka
