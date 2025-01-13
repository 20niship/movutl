#pragma once

#include <movutl/asset/image.hpp>
#include <movutl/core/string.hpp>

namespace mu {

class Entity;

/**
 */
struct TrackLayer {
public:
  FixString name = "Layer";
  bool active = true;
  std::vector<Ref<Entity>> entts;

  Ref<Entity> find_entt(uint32_t guid) const;
  std::string str() const;
  std::string summary() const;
};

class Composition {
public:
  uint32_t guid;
  FixString name = "Main";

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
  Ref<Image> frame_final;
  Ref<Image> frame_edit;
  Ref<Image> frame_temp;

  Vec2d size = {1920, 1080};
  int32_t framerate_nu = 30;
  int32_t framerate_de = 1;

  int32_t fstart = 0; // 表示開始フレーム
  int32_t fend = 200; // 表示終了フレーム
  int32_t frame = 0;  // 現在の表示フレーム

  // ---------- audio ----------
  int16_t* audio_p;
  int32_t audio_n;
  int32_t audio_ch;

  // ---------- track ----------
  std::vector<TrackLayer> layers;

  void resize(int32_t w, int32_t h);

  Composition() = default;
  Composition(const char* name, int32_t w = 1920, int32_t h = 1080, int32_t fps = 30);
  std::string str() const;
  std::string summary() const;

  static Composition* GetActiveComp();
};

} // namespace mu
