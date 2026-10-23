#pragma once

#include <asuka/core/types.hpp>
#include <asuka/utility/callback_slot.hpp>

namespace asuka {

// Core hosts only the process-wide bus every module shares.
/**
 * @brief Process-wide callback slots shared by the odometry core and every
 *        loaded extension module (static objects defined in asuka_core's
 *        callbacks.cpp so all .so files share one instance).
 *
 * Emission contracts. Every slot fires synchronously from the thread named
 * below; observers must not block (move heavy work to their own queue and
 * thread) and may hold the passed ConstPtr for as long as they like --
 * emitters never mutate a payload after emitting it.
 *
 * - on_insert_imu / on_insert_frame
 *     Emitter: AsyncOdometryEstimation, on the odometry worker thread.
 *     Exactly one emission per event delivered to the algorithm, immediately
 *     BEFORE the algorithm's insert_imu()/insert_frame() call, in delivery
 *     order (IMU and frame events interleaved by sensor time). Events dropped
 *     by the bounded input buffers are never emitted, so observers see
 *     exactly the stream the algorithm sees, independent of which odometry
 *     implementation is loaded. Payloads carry the ROS-layer
 *     conversions already applied (acc_scale, imu_time_offset,
 *     time_offset_lidar_to_imu, cloud preprocessing).
 * - on_insert_image
 *     Emitter: AsyncOdometryEstimation, on the odometry worker thread.
 *     Same contract as the two input slots above: one emission per image
 *     delivered to the algorithm, immediately BEFORE the algorithm's
 *     insert_image() call, in delivery order (IMU / image / frame events
 *     interleaved by sensor time). Images dropped by the bounded input
 *     buffer are never emitted. The cv::Mat travels inside ImageData by
 *     ConstPtr; its pixel buffer is shared, never deep-copied.
 * - on_new_frame
 *     Emitter: the odometry implementation, on the odometry worker thread.
 *     Once per frontend keyframe, in keyframe order. Because it shares the
 *     worker thread with the two input slots, the interleaving of all three
 *     is the algorithm's program order -- imu_prediction's forward
 *     integrator relies on IMU samples and their anchor keyframe arriving in
 *     this order. KeyFrame::id counts frontend keyframes; pose-only frames
 *     (id == -1, no cloud) never appear on this slot -- a localization
 *     frontend publishes its map-fused pose on on_odometry_opt like any
 *     backend stream.
 *     Every frame carries a usable imu_bias: the frontend's current bias
 *     estimate (components it does not estimate online stay zero).
 *     KeyFrame::stamp is pinned to the START time of the sensor data that
 *     produced the keyframe (for scan-based frontends: the input scan's
 *     header time; for the zlio frontend: the start of the adaptive
 *     observation window that produced this frame -- zlio is a continuous
 *     point stream with no scan concept). The carried pose/velocity/bias are
 *     the estimator's current state, which typically refers to the window's
 *     end; consumers needing a pose at exactly `stamp` must not assume they
 *     coincide.
 *     imu_prediction re-anchors at this stamp and re-integrates the
 *     [sweep-start, now] IMU window from the sweep-end state.
 * - on_odometry_imu
 *     Emitter: the imu_prediction extension, on its own processing thread,
 *     at IMU rate. Pose-only KeyFrame (id == -1, no cloud, no bias); the
 *     stamp is the emitting stream's own generation time (NOT the
 *     on_new_frame sweep-start contract).
 * - on_odometry_opt
 *     Emitter: backend optimization extensions, on their publish threads, at
 *     their configured publish rate. Pose-only KeyFrame (id == -1, no cloud);
 *     the stamp is the emitting stream's own generation time (NOT the
 *     on_new_frame sweep-start contract).
 */
struct Callbacks {
  static CallbackSlot<void(const ImuData::ConstPtr&)> on_insert_imu;
  static CallbackSlot<void(double stamp, const PointCloudT::ConstPtr& cloud)> on_insert_frame;
  static CallbackSlot<void(const ImageData::ConstPtr&)> on_insert_image;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_new_frame;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_odometry_imu;
  static CallbackSlot<void(const KeyFrame::ConstPtr&)> on_odometry_opt;
};

}  // namespace asuka
