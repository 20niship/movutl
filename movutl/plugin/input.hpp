#pragma once
#include <cstdint>
#include <movutl/asset/project.hpp>

namespace mu {
struct InputInfo {
  enum class Flag : uint32_t {
    Video = 1 << 0,
    Audio = 1 << 1,
    VideoRandomAccess = 1 << 3,
  } flag;

  int32_t rate, scale;
  int32_t n;
  BITMAPINFOHEADER* format;
  int32_t format_size;
  int32_t audio_n;
  WAVEFORMATEX* audio_format;
  int32_t audio_format_size;
  void* handler;
  int32_t reserve[7];
};
} // namespace mu
