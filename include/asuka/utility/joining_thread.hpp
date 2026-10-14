#pragma once

#include <thread>
#include <utility>

namespace asuka {

// std::thread wrapper that joins in the destructor instead of leaking the
// thread (or terminating on move-assignment over a joinable thread, which
// std::thread's move assignment does). Extension modules hold their
// processing threads in one of these: stop() still joins explicitly -- the
// quiesce point AsukaROS relies on before at_exit() -- but forgetting it
// degrades to a join at destruction instead of undefined behavior.
class JoiningThread {
public:
  JoiningThread() = default;
  ~JoiningThread() {
    if (thread.joinable()) thread.join();
  }

  template <typename F, typename... Args>
  explicit JoiningThread(F&& f, Args&&... args) : thread(std::forward<F>(f), std::forward<Args>(args)...) {}

  JoiningThread(const JoiningThread&) = delete;
  JoiningThread& operator=(const JoiningThread&) = delete;

  JoiningThread(JoiningThread&& other) noexcept : thread(std::move(other.thread)) {}
  JoiningThread& operator=(JoiningThread&& other) noexcept {
    if (this != &other) {
      if (thread.joinable()) thread.join();
      thread = std::move(other.thread);
    }
    return *this;
  }

  void join() {
    if (thread.joinable()) thread.join();
  }

  bool joinable() const { return thread.joinable(); }

private:
  std::thread thread;
};

}  // namespace asuka
