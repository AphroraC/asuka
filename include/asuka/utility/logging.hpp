#pragma once

#include <memory>
#include <string>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace asuka {

class Config;

std::shared_ptr<spdlog::logger> get_default_logger();

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

spdlog::level::level_enum log_level_from_string(const std::string& level);

void configure_logging(const Config& config);

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name);

}  // namespace asuka
