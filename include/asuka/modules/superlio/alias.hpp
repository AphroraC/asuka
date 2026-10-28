#pragma once

#include <Eigen/Dense>

namespace asuka {
namespace superlio {

inline const Eigen::Matrix<float, 3, 3> Eye3 = Eigen::Matrix<float, 3, 3>::Identity();
inline const Eigen::Matrix<float, 3, 1> ZeroV3(0.0f, 0.0f, 0.0f);
inline const Eigen::Matrix<float, 3, 1> EyeV3(1.0f, 1.0f, 1.0f);

}  // namespace superlio
}  // namespace asuka
