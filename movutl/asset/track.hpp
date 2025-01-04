#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/core/props.hpp>
#include <movutl/core/ref.hpp>

#include <cstdint>

namespace mu {

inline constexpr size_t MAX_DISPNAME = 64;
inline constexpr size_t MAX_FILTER = 16;

enum BlendType {
  Default = 0,
  Add = 1,
  Sub = 2,
  Mul = 3,
  Div = 4,
  Screen = 5,
  Overlay = 6,
  Darken = 7,
  Lighten = 8,
  HardLight = 9,
};

/**
 */
struct TrackObject {
public:
  uint32_t guid;
  char dispname[MAX_DISPNAME];

  BlendType blend_ = BlendType::Default;

  int32_t layer_disp;
  int32_t frame_begin = 0;
  int32_t frame_end = 1;

  uint32_t layer_ = 0;
  uint32_t compo_ = 0;
  uint32_t group_guid = 0;

  bool active = true;
  bool clipping_up = false;
  bool camera_ctrl = false;

  struct FilterParam {
    uint32_t guid = 0; // フィルタID
    Props props;     // フィルタプロパティ
    bool enabled = true;
  };
  FilterParam filters[MAX_FILTER];
};

} // namespace mu
