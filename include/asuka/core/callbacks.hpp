#pragma once

#include <asuka/core/types.hpp>
#include <asuka/utility/callback_slot.hpp>

namespace asuka {

struct Callbacks {
  static CallbackSlot<void(const ImuData::ConstPtr&)> on_insert_imu;
  static CallbackSlot<void(double stamp, const PointCloudT::ConstPtr& cloud)> on_insert_frame;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_new_frame;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_odometry_imu;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_odometry_opt;
};

}  // namespace asuka
