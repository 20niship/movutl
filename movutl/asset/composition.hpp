#pragma once

#include <movutl/asset/track.hpp>

namespace mu {

/**
 */
struct TrackLayer {
public:
  char dispname[MAX_DISPNAME];
  bool active = true;
  std::vector<TrackObject> objs;
};

struct Composition {
public:
  uint32_t guid;
  char dispname[MAX_DISPNAME];

  enum Flag : uint32_t {
    setting_dialog = 1 << 4,      // そのオブジェクトの設定ダイアログが表示されている
    frame_alpha = 1 << 8,         // frame_edit,frame_tempにアルファチャンネルあり
    fast_preview = 1 << 9,        // 画像処理を間引いて表示
    preprocessing = 1 << 10,      // フィルタの前処理（Filter.Flagのpreprocess参照）
    hide_output_gui = 1 << 11,    // オブジェクト枠の点線などを表示しない
    nesting = 1 << 12,            // シーンオブジェクトなどからフレーム画像取得を行っている
    invert_field_order = 1 << 16, // AviUtl::FilterProcInfo側のフラグ
    invert_interlace = 1 << 17,   // AviUtl::FilterProcInfo側のフラグ
  };
  Flag flag;

  // ---------- video ----------
  Ref<Image> frame_edit;
  Ref<Image> frame_temp;

  Vec2d size = {1920, 1080};
  int32_t scene_line; // scene_maxw
  int32_t scene_maxh;
  int32_t frame;
  int32_t frame_n;

  int32_t framerate_nu = 30;
  int32_t framerate_de = 1;

  // ---------- audio ----------
  int16_t* audio_p;
  int32_t audio_n;
  int32_t audio_ch;


  // ---------- track ----------
  std::vector<TrackLayer> layers;
};

} // namespace mu
