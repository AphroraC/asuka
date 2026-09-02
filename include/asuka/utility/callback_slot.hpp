#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace asuka {

/**
 * @brief Exception handler installed for observer callbacks.
 *
 * A process-wide hook reported to whenever an external callback throws during
 * emission. Keeping it a plain function pointer keeps this header free of any
 * logging framework; asuka_core installs an spdlog-backed handler via
 * set_callback_exception_handler() without dragging spdlog into every TU that
 * includes this template header. Default: nullptr, i.e. thrown exceptions are
 * swallowed silently.
 *
 * @note The hook may be installed only from code, not from a header initializer.
 */
using CallbackExceptionHandler = void (*)(std::exception_ptr) noexcept;

namespace detail {

inline std::atomic<CallbackExceptionHandler>& callback_exception_sink() {
  static std::atomic<CallbackExceptionHandler> sink{nullptr};
  return sink;
}

}  // namespace detail

/**
 * @brief Install (or clear) the process-wide callback exception handler.
 */
inline void set_callback_exception_handler(CallbackExceptionHandler handler) noexcept {
  detail::callback_exception_sink().store(handler, std::memory_order_release);
}

/**
 * @brief Return the currently installed callback exception handler.
 */
inline CallbackExceptionHandler get_callback_exception_handler() noexcept {
  return detail::callback_exception_sink().load(std::memory_order_acquire);
}

/**
 * @brief Report (or swallow) the exception currently being handled.
 *
 * Called from the catch(...) of an emitting thread. Guarded so that even a
 * misbehaving logger installed as the sink cannot escape and terminate the
 * emitting thread.
 */
inline void report_callback_exception() noexcept {
  try {
    CallbackExceptionHandler sink = detail::callback_exception_sink().load(std::memory_order_relaxed);
    if (!sink) {
      return;
    }
    std::exception_ptr ep = std::current_exception();
    if (ep) {
      sink(ep);
    }
  } catch (...) {
    // Nothing sane left to do; the exception is still swallowed below.
  }
}

/**
 * @brief Multi-cast callback slot with runtime add/remove, tuned for
 *        high-frequency emission.
 *
 * Holds a copy-on-write immutable snapshot of the callback list. Emissions
 * only read the currently published snapshot via an atomic load: no mutex, no
 * allocation, no std::function copies on the call path — each callback is
 * dereferenced in place. Mutations (add/remove) serialize on an internal
 * mutex, copy the snapshot, apply the change and republish it; concurrent
 * emitters keep the old snapshot alive for the duration of their call.
 *
 * Stable integer IDs/holes are preserved exactly as before: add() appends and
 * returns the index, remove() leaves a hole, IDs never get reused.
 * Because every emission locks a snapshot at entry, reentrant emit/add/remove
 * (a callback adding/removing subscriptions or emitting the same slot while
 * running) affects subsequent emissions, never the one in flight.
 *
 * Exceptions thrown by observer callbacks are caught per callback and logged
 * via the installed CallbackExceptionHandler (default: swallowed), so one
 * broken observer cannot interrupt the remaining observers or the emitting
 * thread (e.g. the odometry worker).
 *
 * @note The static slot objects that need to be shared across .so boundaries
 *       (see asuka::Callbacks) must be defined in asuka_core's .cpp, NOT as
 *       inline/static members of a header — otherwise each .so keeps its own
 *       instance and callbacks registered from plugins silently vanish.
 *       The exception-handler hook above has the same caveat: set it from the
 *       main DSO if plugins should adopt the same logging sink.
 *
 * @note libstdc++'s shared_ptr atomic free functions only take the lock-free
 *       path when cmpxchg16b is enabled (-mcx16); all TUs compiling this
 *       header must use the same compile strategy, otherwise lock-free
 *       readers and lock-pool writers would share one shared_ptr object in
 *       the same process (undefined behavior).
 */
template <typename Func>
class CallbackSlot {
  using CallbackList = std::vector<std::function<Func>>;

  std::shared_ptr<const CallbackList> snapshot;
  std::atomic<std::size_t> subscriber_count{0};
  // Cache-line isolation: add/remove write mutex while emissions hot-read
  // snapshot/subscriber_count; keeping the mutex on its own cache line
  // avoids false sharing between the mutation and emission paths.
  alignas(64) mutable std::mutex mutex;

  void publish(std::shared_ptr<CallbackList> fresh) {
    std::atomic_store_explicit(
      &snapshot,
      std::shared_ptr<const CallbackList>(std::move(fresh)),
      std::memory_order_release);
  }

public:
  CallbackSlot() = default;
  ~CallbackSlot() = default;

  CallbackSlot(const CallbackSlot&) = delete;
  CallbackSlot& operator=(const CallbackSlot&) = delete;
  CallbackSlot(CallbackSlot&&) = delete;
  CallbackSlot& operator=(CallbackSlot&&) = delete;

  /**
   * @brief Register a callback, returning its stable id (vector index).
   */
  int add(std::function<Func> cb) {
    std::lock_guard<std::mutex> lock(mutex);
    std::shared_ptr<const CallbackList> current = std::atomic_load(&snapshot);
    auto fresh = std::make_shared<CallbackList>();
    if (current) {
      fresh->reserve(current->size() + 1);
      fresh->insert(fresh->end(), current->begin(), current->end());
    }
    const int id = static_cast<int>(fresh->size());
    fresh->emplace_back(std::move(cb));
    publish(std::move(fresh));
    subscriber_count.fetch_add(1, std::memory_order_release);
    return id;
  }

  /**
   * @brief Remove the callback at @p id (leaves a nullptr hole).
   */
  void remove(int id) {
    std::lock_guard<std::mutex> lock(mutex);
    if (id < 0) {
      return;
    }
    const std::size_t idx = static_cast<std::size_t>(id);
    std::shared_ptr<const CallbackList> current = std::atomic_load(&snapshot);
    if (!current || idx >= current->size() || !(*current)[idx]) {
      return;
    }
    auto fresh = std::make_shared<CallbackList>(*current);
    (*fresh)[idx] = nullptr;
    publish(std::move(fresh));
    subscriber_count.fetch_sub(1, std::memory_order_release);
  }

  /**
   * @brief Invoke every non-null callback, forwarding @p args.
   *
   * Lock-free: only atomic-loads the published snapshot. Callbacks are run
   * from the immutable snapshot, so a callback may add/remove subscriptions
   * or emit the same slot recursively without affecting this emission.
   *
   * Rvalue guard (compile time): arguments are forwarded to every observer;
   * a non-trivial rvalue / by-value argument would be moved-from by the first
   * observer and leave moved-from state for the rest. Therefore only lvalue
   * references or trivially-copyable by-value arguments are accepted (for
   * trivially copyable types move == copy, so every observer gets a complete
   * value); pass non-trivial structs as lvalues.
   */
  template <class... Args>
  void call(Args&&... args) const {
    static_assert(
      (... && (std::is_lvalue_reference_v<Args> || std::is_trivially_copyable_v<std::remove_reference_t<Args>>)),
      "CallbackSlot::call forwards arguments to every observer: pass "
      "lvalue references or trivially-copyable values; a non-trivial "
      "rvalue argument would be moved-from after the first observer");
    if (subscriber_count.load(std::memory_order_acquire) == 0) {
      return;
    }
    std::shared_ptr<const CallbackList> snap = std::atomic_load(&snapshot);
    if (!snap) {
      return;
    }
    for (const auto& cb : *snap) {
      if (!cb) {
        continue;
      }
      try {
        cb(std::forward<Args>(args)...);
      } catch (...) {
        report_callback_exception();
      }
    }
  }

  /**
   * @brief Operator form of call().
   */
  template <class... Args>
  void operator()(Args&&... args) const {
    call(std::forward<Args>(args)...);
  }

  /**
   * @brief True if at least one non-null callback is registered.
   */
  explicit operator bool() const { return subscriber_count.load(std::memory_order_acquire) != 0; }
};

}  // namespace asuka
