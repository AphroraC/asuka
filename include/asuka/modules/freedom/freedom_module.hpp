#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/circular_buffer.hpp>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/joining_thread.hpp>
#include <asuka/utility/logging.hpp>

#include <asuka/modules/freedom/freedom.h>
#include <asuka/modules/freedom/freedom_viz.hpp>

namespace asuka {

// Online dynamic object removal on the frontend keyframe stream (port of
// FreeDOM, RA-L 2025). Frontend-agnostic by construction: it consumes
// Callbacks::on_new_frame -- the (cloud_imu, T_world_imu) pair of every
// processed scan, whatever odometry plugin emitted it -- and feeds them to
// the upstream FreeDOM pipeline unchanged (upstream input contract:
// sensor-frame cloud + world-from-sensor transform; the asuka KeyFrame pair
// matches it with the undistorted IMU-frame scan and the frontend pose, no
// TF involved).
//
// Pure-algorithm extension module (no ROS dependency): the upstream
// visualization topics are replaced by the on_freedom_* callback slots and
// the ROS publishing itself is rviz_viewer's job. Map saving is configured
// in the "freedom" block of config_extensions (enable_map_saving /
// map_saving_path) and runs from at_exit().
class FreedomModule : public ExtensionModule {
public:
  FreedomModule();
  ~FreedomModule() override;
  void stop() override;
  void at_exit(const std::string& dump_path) override;

private:
  void on_new_frame(const KeyFrame::ConstPtr& frame);
  void processing_thread();
  void process_frame(const KeyFrame::ConstPtr& frame);

  // Stage callbacks handed to the upstream FreeDOM class. They fire inside
  // pointcloud_integrate on the processing thread while the scan/map
  // structures are alive (upstream resets them right after), so the payload
  // builders snapshot them synchronously -- nothing escapes by reference.
  void emit_scan_removal(const freedom::ScanMap& scan);
  void emit_raycast_enhancement(const freedom::DepthImage& image);
  void emit_map_removal(const freedom::MRMap& map);

  static constexpr std::size_t queue_capacity = 512;

  freedom::FreeDOM freedom;
  FreedomVisualizer visualizer;

  bool enable_map_saving{false};
  std::string map_saving_path{};

  // RAII subscription: destruction removes the callback. Declared before
  // the processing thread so destruction joins the thread first, then
  // unsubscribes.
  CallbackSubscription<void(const KeyFrame::ConstPtr&)> on_new_frame_sub;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<KeyFrame::ConstPtr> queue{queue_capacity};
  std::atomic<bool> kill_switch{false};
  JoiningThread processing_thread_obj;
  std::size_t dropped_items{0};
  // Stamp of the frame currently being integrated; the upstream stage
  // callbacks carry no timestamp, so it is captured per frame on the same
  // (processing) thread.
  double current_stamp{0.0};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka
