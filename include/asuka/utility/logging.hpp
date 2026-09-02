#pragma once

#include <memory>
#include <string>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace asuka {

std::shared_ptr<spdlog::logger> get_default_logger();

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size = 128);

spdlog::level::level_enum log_level_from_string(const std::string& level);

void configure_logging();

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name);

}  // namespace asuka
