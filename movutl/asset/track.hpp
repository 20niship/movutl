#pragma once

#include <movutl/core/anim.hpp>
#include <movutl/core/props.hpp>
#include <movutl/core/ref.hpp>

#include <cstdint>

namespace mu {

inline constexpr size_t MAX_DISPNAME = 64;
inline constexpr size_t MAX_FILTER = 16;

enum BlendType { // MPROPERTY(name="合成モード")
  Blend_Alpha = 0,
  Blend_Add = 1,
  Blend_Sub = 2,
  Blend_Mul = 3,
  Blend_Div = 4,
  Blend_Screen = 5,
  Blend_Overlay = 6,
  Blend_Darken = 7,
  Blend_Lighten = 8,
  Blend_HardLight = 9,
};

struct FilterPluginTable;

/**
 * あるレイヤ上に存在する一つのオブジェクト
 * これはEntityのデータの中に含まれる
 */
struct TrackObject {
public:
  uint32_t guid;

  int fstart = -1;                // MPROPERTY(name="開始位置(frame)" hidden_inspector=true)
  int fend = -1;                  // MPROPERTY(name="終了位置(frame)" hidden_inspector=true)
  Vec2 anchor;                    // MPROPERTY(name="アンカー", viewer_anchor=true, position=true)
  BlendType blend_ = Blend_Alpha; // MPROPERTY(name="合成モード")
  uint32_t group_guid = 0;        // MPROPERTY(name="グループID", desc="グループ化されている時のグループID", hidden_inspector=true)
  bool active_ = true;            // MPROPERTY(name="アクティブ", desc="オブジェクトが有効かどうか")
  bool solo_ = false;             // MPROPERTY(name="ソロモード", desc="(音声のみ)他のレイヤを非表示にする")
  bool clipping_up = false;       // MPROPERTY(name="上レイヤでクリッピング",  hidden_inspector=true)
  bool camera_ctrl = false;       // MPROPERTY(name="カメラ制御", desc="カメラ制御の対象", hidden_inspector=true)

  struct FilterParam {
    FilterPluginTable* plg_ = nullptr;
    uint32_t guid = 0; // フィルタID
    AnimProps props;   // フィルタプロパティ
    bool enabled = true;
  };
  std::vector<FilterParam> filters;

  bool visible(int frame) const { return fstart <= frame && frame <= fend && active_; }

  PropsInfo getPropsInfo() const;    // MUFUNC_AUTOGEN
  Props getProps() const;            // MUFUNC_AUTOGEN
  void setProps(const Props& props); // MUFUNC_AUTOGEN
};

} // namespace mu
