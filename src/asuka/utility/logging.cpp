#include <algorithm>
#include <cctype>
#include <filesystem>

#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

std::shared_ptr<spdlog::logger> get_default_logger() {
  return spdlog::default_logger();
}

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger) {
  spdlog::set_default_logger(logger);
}

namespace {
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink;
}

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size) {
  if (!ringbuffer_sink) {
    ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(buffer_size);
  }
  return ringbuffer_sink;
}

spdlog::level::level_enum log_level_from_string(const std::string& level) {
  std::string normalized = level;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (normalized == "trace") return spdlog::level::trace;
  if (normalized == "debug") return spdlog::level::debug;
  if (normalized == "info") return spdlog::level::info;
  if (normalized == "warn" || normalized == "warning") return spdlog::level::warn;
  if (normalized == "error") return spdlog::level::err;
  if (normalized == "critical") return spdlog::level::critical;
  if (normalized == "off") return spdlog::level::off;
  return spdlog::level::info;
}

void configure_logging() {
  const Config& config = *GlobalConfig::instance();
  const auto level = log_level_from_string(config.param<std::string>("logging", "logging_level", std::string("info")));
  spdlog::set_level(level);
  if (spdlog::default_logger()) {
    spdlog::default_logger()->set_level(level);
  }
  spdlog::apply_all([level](const std::shared_ptr<spdlog::logger>& logger) { logger->set_level(level); });

  const bool console_output = config.param<bool>("logging", "console_output", true);
  if (!console_output && spdlog::default_logger()) {
    auto& sinks = spdlog::default_logger()->sinks();
    sinks.erase(
      std::remove_if(
        sinks.begin(),
        sinks.end(),
        [](const auto& sink) {
          return dynamic_cast<spdlog::sinks::stdout_color_sink_mt*>(sink.get()) != nullptr ||
                 dynamic_cast<spdlog::sinks::stdout_color_sink_st*>(sink.get()) != nullptr;
        }),
      sinks.end());
  }
}

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name) {
  std::shared_ptr<spdlog::logger> logger = spdlog::get(module_name);
  if (logger) {
    return logger;
  }

  const Config& config = *GlobalConfig::instance();
  const std::string log_dir = config.param<std::string>("logging", "logging_dir", std::string("/tmp"));
  const std::string log_filename = (module_name == "asuka") ? "main" : module_name;

  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
  }

  const auto logging_level =
    log_level_from_string(config.param<std::string>("logging", "logging_level", std::string("info")));
  const bool console_output = config.param<bool>("logging", "console_output", true);
  const auto console_level =
    log_level_from_string(config.param<std::string>("logging", "console_level", std::string("info")));

  logger = std::make_shared<spdlog::logger>(module_name);
  spdlog::register_logger(logger);

  auto ring_sink = get_ringbuffer_sink();
  ring_sink->set_level(logging_level);
  logger->sinks().push_back(ring_sink);

  if (console_output) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(console_level);
    logger->sinks().push_back(console_sink);
  }

  if (config.param<bool>("logging", "file_output", true)) {
    std::shared_ptr<spdlog::sinks::sink> file_sink;
    if (config.param<bool>("logging", "rotate_logs", true)) {
      const size_t max_file_size_kb = config.param<int>("logging", "max_file_size_kb", 8192);
      const size_t max_file_size_bytes = max_file_size_kb * 1024;
      const size_t max_files = config.param<int>("logging", "max_files", 10);
      file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_dir + "/asuka_" + log_filename + ".log",
        max_file_size_bytes,
        max_files);
    } else {
      file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_dir + "/asuka_" + log_filename + ".log", true);
    }
    file_sink->set_level(logging_level);
    logger->sinks().push_back(file_sink);
  }

  auto min_level = logging_level;
  if (console_output && console_level < min_level) {
    min_level = console_level;
  }
  logger->set_level(min_level);

  return logger;
}

}  // namespace asuka
