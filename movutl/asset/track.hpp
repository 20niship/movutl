#pragma once

#include <movutl/core/anim.hpp>
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

struct FilterPluginTable;

/**
 * あるレイヤ上に存在する一つのオブジェクト
 * これはEntityのデータの中に含まれる
 */
struct TrackObject {
public:
  uint32_t guid;

  int fstart = -1;    // スタート位置(frame)
  int fend = -1;      // 最終位置(frame)
  float anchor_x = 0; // 基準点のX座標　（デフォルトでは width / 2）
  float anchor_y = 0;

  // TODO: x, yを変化させるときはどうするの？
  float move_x = 0; // 基準点のX座標の移動距離（デフォルトでは0）
  float move_y = 0;

  BlendType blend_ = BlendType::Default;

  uint32_t group_guid = 0; // グループ化されている時のグループID

  bool active_ = true;
  bool solo_ = false;       // (audioのみ)ソロモード
  bool clipping_up = false; // 上のレイヤーをクリッピングする
  bool camera_ctrl = false; // カメラ制御の対象
  bool alpha_ = false;      // アルファを使って合成する

  struct FilterParam {
    FilterPluginTable* plg_ = nullptr;
    uint32_t guid = 0; // フィルタID
    AnimProps props;   // フィルタプロパティ
    bool enabled = true;
  };
  std::vector<FilterParam> filters;

  bool visible(int frame) const { return fstart <= frame && frame <= fend && active_; }
};

} // namespace mu
