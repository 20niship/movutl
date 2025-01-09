#pragma once
#include <loguru.hpp>
#include <string>

namespace mu {

enum LogLevel { TRACE = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, FATAL = 5, OFF = 6 };

namespace detail {

void init_logger();

std::string get_log_file_path();

} // namespace detail
} // namespace mu
