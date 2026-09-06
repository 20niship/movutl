#pragma once

#include <cutil/prop.hpp>
#include <cutil/ref.hpp>
#include <movutl/core/anim.hpp>

#include <cstdint>

namespace mu {

using cutil::Ref;

inline constexpr size_t MU_MAX_NAME = 32;
inline constexpr size_t MAX_FILTER  = 16;

enum BlendType { // MPROPERTY(name="合成モード")
  Blend_Alpha     = 0,
  Blend_Add       = 1,
  Blend_Sub       = 2,
  Blend_Mul       = 3,
  Blend_Div       = 4,
  Blend_Screen    = 5,
  Blend_Overlay   = 6,
  Blend_Darken    = 7,
  Blend_Lighten   = 8,
  Blend_HardLight = 9,
};

struct FilterPluginTable;

// 一つのEntity(トラック上のオブジェクト)に適用されているフィルタ1個分のパラメータ
struct FilterParam {
  FilterPluginTable* plg_ = nullptr;
  uint32_t guid           = 0; // フィルタID
  AnimProps props;             // フィルタプロパティ
  bool enabled = true;
  // 音声フィルタ用のトラックオブジェクト固有DSP状態(ディレイライン等)。fn_proc(&instance_state, ...)としてfp引数に渡される
  void* instance_state = nullptr;
  FilterParam()        = default;
  FilterParam(FilterPluginTable* plg, uint32_t guid) : plg_(plg), guid(guid) {}
  ~FilterParam() = default;
};

} // namespace mu
