//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_IMU_H
#define LIGHTNING_IMU_H

#include "common/eigen_types.h"

namespace lightning {

struct IMU {
    double timestamp = 0;
    Vec3d angular_velocity = Vec3d::Zero();
    Vec3d linear_acceleration = Vec3d::Zero();
};

using IMUPtr = std::shared_ptr<IMU>;

}  // namespace lightning

#endif  // LIGHTNING_IMU_H
