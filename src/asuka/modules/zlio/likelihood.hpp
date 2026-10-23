/**
 * @file likelihood.hpp
 * @brief 累计同面观测并消去地图平面的共享参数噪声。
 */
#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "moments.hpp"
#include <asuka/modules/zlio/types.hpp>
namespace asuka {
namespace zlio {

/// 展开三维平面参数的紧凑对称协方差。
inline Mat3 expandPlaneNoise(const std::array<double, 6>& p) {
    Mat3 C;
    C << p[0], p[1], p[2], p[1], p[3], p[4], p[2], p[4], p[5];
    return C;
}

/// 保存单个共享平面对位姿的信息与梯度贡献。
struct PoseFactor {
    Mat6 information;
    Vec6 gradient;
};

/// 由同面充分统计及平面先验构造三维共享观测因子。
class MappedPlaneFactor {
    Mat3 root_;
    Vec3 residual_;
    Eigen::LLT<Mat3> factor_;

public:
    /// 在三维有效子空间中分解观测信息并叠加平面协方差。
    MappedPlaneFactor(const Mat3& S, const Vec3& b, const Mat3& covariance) {
        const Eigen::LLT<Mat3> sf(S);
        if (sf.info() == Eigen::Success &&
            sf.matrixL().toDenseMatrix().diagonal().minCoeff() >
                1e-6 * sf.matrixL().toDenseMatrix().diagonal().maxCoeff()) {
            root_ = sf.matrixL().transpose();
            residual_ = sf.matrixL().solve(b);
        } else {
            Eigen::SelfAdjointEigenSolver<Mat3> eig(S);
            const Vec3& values = eig.eigenvalues();
            const double tol =
                64 * std::numeric_limits<double>::epsilon() * std::max(1e-30, values.maxCoeff());
            Vec3 root = Vec3::Zero(), inv = Vec3::Zero();
            for (int k = 0; k < 3; ++k)
                if (values[k] > tol) {
                    root[k] = std::sqrt(values[k]);
                    inv[k] = 1 / root[k];
                }
            root_ = root.asDiagonal() * eig.eigenvectors().transpose();
            residual_ = inv.asDiagonal() * eig.eigenvectors().transpose() * b;
        }
        factor_.compute(Mat3::Identity() + root_ * covariance * root_.transpose());
        if (factor_.info() != Eigen::Success)
            throw std::runtime_error("invalid shared plane likelihood");
    }
    /// 将三维共享观测转换为六维位姿信息及梯度。
    PoseFactor pose(const Eigen::Matrix<double, 3, 6>& transform) const {
        const Eigen::Matrix<double, 3, 6> H = root_ * transform;
        return {H.transpose() * factor_.solve(H), H.transpose() * factor_.solve(residual_)};
    }
};

/// 保存完成关联后的点面几何及方差分量。
struct CompactPlaneRow {
    Vec3 normal, plane_jacobian;
    const std::array<double, 6>* covariance = nullptr;
    double residual = 0, independent = 0, total = 0;
};
/// 保存同面分组首个观测的位姿及平面 Jacobian。
struct SharedPlaneRow {
    Vec6 jacobian;
    Vec3 plane_jacobian;
    const std::array<double, 6>* covariance = nullptr;
    double residual = 0, independent = 0, total = 0;
};
/// 按首次出现顺序归并共享平面观测并累计其充分统计。
class SharedPlaneAccumulator {
    struct Slot {
        const void* key = nullptr;
        std::size_t group = 0;
    };
    struct Group {
        SharedPlaneRow first;
        std::size_t moment = 0;
        bool shared = false;
    };
    struct Moments {
        std::array<double, 6> symmetric{};
        /// 展开三维参数空间的信息矩阵。
        Mat3 S() const {
            Mat3 value;
            for (std::size_t k = 0; k < 6; ++k) {
                auto ij = plane::kSymmetricPairs[k];
                value(ij[0], ij[1]) = value(ij[1], ij[0]) = symmetric[k];
            }
            return value;
        }
        Vec3 b = Vec3::Zero();
        /// 累计独立噪声加权的参数信息与残差梯度。
        template <class Row>
        void add(const Row& row) {
            const auto outer =
                row.plane_jacobian * row.plane_jacobian.transpose() / row.independent;
            for (std::size_t k = 0; k < 6; ++k) {
                auto ij = plane::kSymmetricPairs[k];
                symmetric[k] += outer(ij[0], ij[1]);
            }
            b += row.plane_jacobian * (row.residual / row.independent);
        }
    };
    std::vector<Slot> slots_;
    std::vector<Group> groups_;
    std::vector<Moments> moments_;
    std::size_t count_ = 0, maximum_rows_ = 0;

public:
    /// 清空本轮分组并按最大观测数量准备哈希工作区。
    void reset(std::size_t maximum_rows) {
        count_ = 0;
        maximum_rows_ = maximum_rows;
        std::size_t capacity = 1;
        while (maximum_rows > capacity / 2) {
            if (capacity > slots_.max_size() / 2)
                throw std::length_error("shared plane workspace exceeds capacity");
            capacity *= 2;
        }
        // 释放明显超过当前需求的历史工作区容量。
        if (slots_.capacity() / 4 > std::max<std::size_t>(capacity, 1024)) {
            std::vector<Slot>().swap(slots_);
            std::vector<Group>().swap(groups_);
            std::vector<Moments>().swap(moments_);
        }
        slots_.assign(capacity, Slot{});
        groups_.clear();
        moments_.clear();
    }
    /// 将观测归入共享平面，仅为新平面计算位姿 Jacobian。
    template <class Row, class Jacobian>
    void add(const Row& row, const Jacobian& make_jacobian) {
        if (!row.covariance)
            throw std::invalid_argument("map backend does not expose shared plane covariance");
        if (count_ >= maximum_rows_)
            throw std::logic_error("shared plane workspace row bound exceeded");
        const void* key = row.covariance;
        std::uint64_t hash = reinterpret_cast<std::uintptr_t>(key);
        hash ^= hash >> 33;
        hash *= 0xff51afd7ed558ccdULL;
        hash ^= hash >> 33;
        std::size_t at = hash & (slots_.size() - 1);
        while (slots_[at].key && slots_[at].key != key)
            at = (at + 1) & (slots_.size() - 1);
        if (!slots_[at].key) {
            const auto index = groups_.size();
            groups_.push_back({{make_jacobian(), row.plane_jacobian, row.covariance, row.residual,
                                row.independent, row.total},
                               0,
                               false});
            slots_[at] = {key, index};
        } else {
            auto& group = groups_[slots_[at].group];
            if (!group.shared) {
                const auto index = moments_.size();
                moments_.emplace_back();
                moments_.back().add(group.first);
                group.moment = index;
                group.shared = true;
            }
            moments_[group.moment].add(row);
        }
        ++count_;
    }
    /// 消去各组平面参数并按输入顺序累计位姿信息。
    Linearization finish() const {
        Linearization out;
        out.count = count_;
        const auto add = [&](const Mat6& I, const Vec6& g) {
            out.information.topLeftCorner<6, 6>() += I;
            out.gradient.head<6>() += g;
        };
        // 多点组消去共享参数，单点组直接累计标量似然。
        for (const auto& group : groups_) {
            const auto& first = group.first;
            if (group.shared) {
                const auto& moments = moments_[group.moment];
                const MappedPlaneFactor plane(moments.S(), moments.b,
                                              expandPlaneNoise(*first.covariance));
                const Vec3 normal = first.jacobian.head<3>(), u = normal.unitOrthogonal(),
                           v = normal.cross(u);
                Eigen::Matrix<double, 3, 6> T = Eigen::Matrix<double, 3, 6>::Zero();
                T.block<1, 3>(0, 3) = u.cross(normal).transpose();
                T.block<1, 3>(1, 3) = v.cross(normal).transpose();
                T.row(2) = first.plane_jacobian[0] * T.row(0) + first.plane_jacobian[1] * T.row(1) -
                           first.jacobian.transpose();
                const auto contribution = plane.pose(T);
                add(contribution.information, contribution.gradient);
            } else {
                add(first.jacobian * first.jacobian.transpose() / first.total,
                    first.jacobian * (first.residual / first.total));
            }
        }
        out.information = .5 * (out.information + out.information.transpose()).eval();
        return out;
    }
};

}  // namespace zlio
}  // namespace asuka
