#pragma once
#include <vector>
//
#include <movutl/core/defines.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/props.hpp>

namespace mu {

struct Config {
public:
  MOVUTL_DECLARE_SINGLETON(Config);
  Config() = default;
  ~Config() = default;

  Vec2d max_size = {3000, 3000};
  int max_frame = 320000;
  int cache_frames = 10;
  std::vector<std::string> plugin_search_paths;
  bool log_to_file = false;
  std::string log_filename = "log.txt";
  LogLevel log_level = LogLevel::INFO;

  static void Load();
  static void Save();
  static void Reload();
};
} // namespace mu
