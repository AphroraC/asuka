#include <asuka/core/callbacks.hpp>

#include <spdlog/spdlog.h>

namespace asuka {

CallbackSlot<void(const ImuData::ConstPtr&)> Callbacks::on_insert_imu;
CallbackSlot<void(double stamp, const PointCloudT::ConstPtr& cloud)> Callbacks::on_insert_frame;
CallbackSlot<void(const KeyFrame::ConstPtr&)> Callbacks::on_new_frame;
CallbackSlot<void(const KeyFrame::ConstPtr&)> Callbacks::on_odometry_imu;
CallbackSlot<void(const KeyFrame::ConstPtr&)> Callbacks::on_odometry_opt;

namespace {

struct CallbackExceptionLogger {
  CallbackExceptionLogger() {
    set_callback_exception_handler([](std::exception_ptr ep) noexcept {
      try {
        std::rethrow_exception(ep);
      } catch (const std::exception& e) {
        spdlog::warn("[callbacks] observer exception: {}", e.what());
      } catch (...) {
        spdlog::warn("[callbacks] observer exception: unknown");
      }
    });
  }
};

CallbackExceptionLogger g_callback_exception_logger;

}  // namespace

}  // namespace asuka
