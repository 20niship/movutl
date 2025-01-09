#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdint.h>
#include <time.h>

namespace mu::detail {

static std::string get_logger_filename() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  return ss.str();
}

void init_logger() {
  auto c = Config::Get();
  if(c->log_to_file) {
    auto fname = get_logger_filename();
    try {
      std::filesystem::create_directory("logs");
      c->log_filename = "./logs/" + fname + ".log";
    } catch(std::exception& e) {
      c->log_filename = "";
    }
    if(!c->log_filename.empty()) loguru::add_file(c->log_filename.c_str(), loguru::Append, 1);
  }

  loguru::g_preamble_date = false;
  loguru::g_preamble_uptime = false;
  loguru::g_preamble_thread = false;
  loguru::g_preamble_file = false;

  switch(c->log_level) {
    case LogLevel::TRACE: loguru::g_stderr_verbosity = 2; break;
    case LogLevel::DEBUG: loguru::g_stderr_verbosity = 1; break;
    case LogLevel::INFO: loguru::g_stderr_verbosity = loguru::Verbosity_INFO; break;
    case LogLevel::WARNING: loguru::g_stderr_verbosity = loguru::Verbosity_WARNING; break;
    case LogLevel::ERROR: loguru::g_stderr_verbosity = loguru::Verbosity_ERROR; break;
    case LogLevel::FATAL: loguru::g_stderr_verbosity = loguru::Verbosity_FATAL; break;
    case LogLevel::OFF: loguru::g_stderr_verbosity = loguru::Verbosity_OFF; break;
  }
}

std::string get_log_file_path() {
  auto c = Config::Get();
  return c->log_filename;
}

} // namespace mu::detail
