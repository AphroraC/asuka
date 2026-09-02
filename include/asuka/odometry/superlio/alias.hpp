#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace asuka {
namespace superlio {

using Scalar = float;

using V2 = Eigen::Matrix<Scalar, 2, 1>;
using V3 = Eigen::Matrix<Scalar, 3, 1>;
using V4 = Eigen::Matrix<Scalar, 4, 1>;
using V5 = Eigen::Matrix<Scalar, 5, 1>;
using V6 = Eigen::Matrix<Scalar, 6, 1>;
using V7 = Eigen::Matrix<Scalar, 7, 1>;
using V8 = Eigen::Matrix<Scalar, 8, 1>;
using V9 = Eigen::Matrix<Scalar, 9, 1>;
using V12 = Eigen::Matrix<Scalar, 12, 1>;
using V15 = Eigen::Matrix<Scalar, 15, 1>;
using V17 = Eigen::Matrix<Scalar, 17, 1>;
using V18 = Eigen::Matrix<Scalar, 18, 1>;
using VX = Eigen::Matrix<Scalar, -1, 1>;

using VV3 = std::vector<V3, Eigen::aligned_allocator<V3>>;
using VV4 = std::vector<V4, Eigen::aligned_allocator<V4>>;

using M1 = Eigen::Matrix<Scalar, 1, 1>;
using M2 = Eigen::Matrix<Scalar, 2, 2>;
using M3 = Eigen::Matrix<Scalar, 3, 3>;
using M4 = Eigen::Matrix<Scalar, 4, 4>;
using M5 = Eigen::Matrix<Scalar, 5, 5>;
using M6 = Eigen::Matrix<Scalar, 6, 6>;
using M12 = Eigen::Matrix<Scalar, 12, 12>;
using M17 = Eigen::Matrix<Scalar, 17, 17>;
using M18 = Eigen::Matrix<Scalar, 18, 18>;

using M3_2 = Eigen::Matrix<Scalar, 3, 2>;
using M2_3 = Eigen::Matrix<Scalar, 2, 3>;

using Quat = Eigen::Quaternion<Scalar>;

using V2d = Eigen::Matrix<double, 2, 1>;
using V3d = Eigen::Matrix<double, 3, 1>;
using V4d = Eigen::Matrix<double, 4, 1>;
using V5d = Eigen::Matrix<double, 5, 1>;
using V6d = Eigen::Matrix<double, 6, 1>;
using V7d = Eigen::Matrix<double, 7, 1>;
using V8d = Eigen::Matrix<double, 8, 1>;
using V9d = Eigen::Matrix<double, 9, 1>;
using V12d = Eigen::Matrix<double, 12, 1>;
using V15d = Eigen::Matrix<double, 15, 1>;
using V18d = Eigen::Matrix<double, 18, 1>;

using M1d = Eigen::Matrix<double, 1, 1>;
using M2d = Eigen::Matrix<double, 2, 2>;
using M3d = Eigen::Matrix<double, 3, 3>;
using M4d = Eigen::Matrix<double, 4, 4>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M18d = Eigen::Matrix<double, 18, 18>;

using Quatd = Eigen::Quaternion<double>;

const M3 Eye3 = M3::Identity();
const V3 ZeroV3(0, 0, 0);
const V3 EyeV3(1, 1, 1);

}  // namespace superlio
}  // namespace asuka
