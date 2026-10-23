// 导航误差坐标的性质由独立差分和解析物理量验证，不冻结实现中间值。
#include <asuka/modules/zlio/state.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <random>

namespace asuka {
namespace zlio {
namespace {
State referenceState() {
    State state;
    state.pose.rotation = expRotation(Vec3(.2, -.3, .4));
    state.pose.position = Vec3(2, -3, 5);
    state.velocity = Vec3(.7, -.2, .5);
    state.gyro_bias = Vec3(.01, -.02, .03);
    state.accel_bias = Vec3(-.1, .2, .05);
    return state;
}

TEST(NavigationState, RetractionAndDifferenceAreInverseWithinRotationBranch) {
    const State reference = referenceState();
    std::mt19937 random(20261020);
    std::uniform_real_distribution<double> sample(-.15, .15);
    for (int trial = 0; trial < 40; ++trial) {
        SCOPED_TRACE(trial);
        Vec15 increment;
        for (int axis = 0; axis < 15; ++axis)
            increment[axis] = sample(random);
        EXPECT_LT((difference(retract(reference, increment), reference) - increment).norm(), 1e-11);
    }
}

TEST(NavigationState, PointJacobianMatchesPerturbedWorldGeometry) {
    const State state = referenceState();
    const Vec3 point_body(1, 2, -1);
    const PointJacobian analytic = pointJacobian(state.pose.apply(point_body));
    constexpr double step = 1e-6;
    for (int axis = 0; axis < 15; ++axis) {
        SCOPED_TRACE(axis);
        Vec15 delta = Vec15::Zero();
        delta[axis] = step;
        const Vec3 derivative = (retract(state, delta).pose.apply(point_body) -
                                 retract(state, -delta).pose.apply(point_body)) /
                                (2 * step);
        EXPECT_LT((derivative - analytic.col(axis)).norm(), 1e-8);
    }
}

TEST(NavigationState, PriorAndResetJacobiansMatchCentralDifferences) {
    const State reference = referenceState();
    const Vec15 increment = Vec15::LinSpaced(-.06, .08);
    const State state = retract(reference, increment);
    const Mat15 prior = priorErrorJacobian(state, reference);
    const Mat15 reset = errorResetJacobian(increment);
    constexpr double step = 1e-6;
    for (int axis = 0; axis < 15; ++axis) {
        SCOPED_TRACE(axis);
        Vec15 delta = Vec15::Zero();
        delta[axis] = step;
        const Vec15 prior_column = (difference(retract(state, delta), reference) -
                                    difference(retract(state, -delta), reference)) /
                                   (2 * step);
        const Vec15 reset_column = (difference(retract(reference, increment + delta), state) -
                                    difference(retract(reference, increment - delta), state)) /
                                   (2 * step);
        EXPECT_LT((prior_column - prior.col(axis)).norm(), 2e-8);
        EXPECT_LT((reset_column - reset.col(axis)).norm(), 2e-8);
    }
}

TEST(NavigationCovariance, InversionPreservesCoupledPositiveDefiniteMatrix) {
    Mat15 basis = Mat15::Identity();
    basis(0, 7) = .4;
    basis(3, 12) = -.2;
    const Mat15 covariance = basis * basis.transpose();
    ASSERT_TRUE(numericallyPositiveCovariance(covariance));
    EXPECT_LT((covariance * inversePositiveDefinite(covariance) - Mat15::Identity()).norm(), 1e-12);
}

TEST(NavigationCovariance, RejectsSingularIndefiniteAndNonfiniteInputs) {
    Mat15 covariance = Mat15::Identity();
    covariance(0, 0) = 0;
    EXPECT_FALSE(numericallyPositiveCovariance(covariance));
    EXPECT_THROW(inversePositiveDefinite(covariance), NumericalError);
    covariance(0, 0) = -1;
    EXPECT_FALSE(numericallyPositiveCovariance(covariance));
    covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(numericallyPositiveCovariance(covariance));
}
}  // namespace
}  // namespace zlio
}  // namespace asuka
