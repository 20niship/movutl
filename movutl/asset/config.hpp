#pragma once
#include <movutl/core/defines.hpp>
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
};
} // namespace mu
