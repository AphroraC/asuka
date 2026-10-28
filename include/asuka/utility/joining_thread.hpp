#pragma once

#include <thread>
#include <utility>

namespace asuka {

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
