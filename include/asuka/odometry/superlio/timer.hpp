#pragma once

#include <chrono>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace asuka {
namespace superlio {

class Timer {
public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name = name;
      time_usage_ms.emplace_back(time_usage);
    }
    std::string func_name;
    std::vector<double> time_usage_ms;
  };

  template <class F>
  void evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
    if (records.find(func_name) != records.end()) {
      records[func_name].time_usage_ms.emplace_back(time_used);
    } else {
      records.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  void print_all() const {
    spdlog::info(">>> ===== Printing run time =====");
    for (const auto& r : records) {
      double avg = std::accumulate(r.second.time_usage_ms.begin(), r.second.time_usage_ms.end(), 0.0) /
                   double(r.second.time_usage_ms.size());
      spdlog::info("> [ {} ] average time usage: {} ms , called times: {}", r.first, avg,
                  r.second.time_usage_ms.size());
    }
  }

  void clear() { records.clear(); }

private:
  std::map<std::string, TimerRecord> records;
};

}  // namespace superlio
}  // namespace asuka
