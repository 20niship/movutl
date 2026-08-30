#pragma once

#include <cutil/prop.hpp>
#include <cutil/string.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/render2d/frame_cache.hpp>
#include <mutex>

namespace mu {

class Entity;

/**
 */
struct TrackLayer {
public:
  cutil::Str name{"Layer"};
  bool active = true;
  std::vector<Ref<Entity>> entts;

  Ref<Entity> find_entt(uint32_t guid) const;
  std::string str() const;
  std::string summary() const;

  // name/activeのみを対象とする(enttsはEntityへのRefのリストでguid参照による再構築が必要なためProject::Save/Loadで別途扱う)
  const cutil::PropInfo* getPropsInfo() const;
  cutil::Prop getProps() const;
  void setProps(const cutil::Prop& props);
};

class Composition {
public:
  uint32_t guid;
  cutil::Str name{"Main"};

  enum Flag : uint32_t {
    setting_dialog     = 1 << 4,  // そのオブジェクトの設定ダイアログが表示されている
    frame_alpha        = 1 << 8,  // レンダリング結果にアルファチャンネルあり
    fast_preview       = 1 << 9,  // 画像処理を間引いて表示
    preprocessing      = 1 << 10, // フィルタの前処理（Filter.Flagのpreprocess参照）
    hide_output_gui    = 1 << 11, // オブジェクト枠の点線などを表示しない
    nesting            = 1 << 12, // シーンオブジェクトなどからフレーム画像取得を行っている
    invert_field_order = 1 << 16, // AviUtl::FilterProcInfo側のフラグ
    invert_interlace   = 1 << 17, // AviUtl::FilterProcInfo側のフラグ
  };
  Flag flag = (Flag)0;

  Vec2d size       = {1920, 1080};
  float framerate  = 30.0f;
  int32_t bg_color = (int32_t)0xFF000000; // フレーム画像の背景色(ImU32/RGBA)、デフォルトは黒不透明

  int32_t fstart = 0;   // 表示開始フレーム
  int32_t fend   = 200; // 表示終了フレーム
  int32_t frame  = 0;   // 現在の表示フレーム

  // ---------- audio ----------
  int16_t* audio_p;
  int32_t audio_n;
  int32_t audio_ch;

  // ---------- track ----------
  std::vector<TrackLayer> layers;

  // レイヤー/フレーム範囲などレンダリング結果に影響する状態を読み書きする際のロック
  std::mutex mtx;
  // フレーム単位のレンダリング結果キャッシュ(バックグラウンドレンダーワーカーが書き込む)
  FrameCache cache;

  void resize(int32_t w, int32_t h);

  Composition() = default;
  Composition(const char* name, int32_t w = 1920, int32_t h = 1080, int32_t fps = 30);
  std::string str() const;
  std::string summary() const;

  static Composition* GetActiveComp();

  const cutil::PropInfo* getPropsInfo() const;
  cutil::Prop getProps() const;
  void setProps(const cutil::Prop& props);

  int insertable_layer_index() const;
  void insert_entity(Ref<Entity> entt, int layer = -1);

  // 現在フレームをバックグラウンドキューを使わずその場で同期レンダリングして取得する(キャッシュ済みならそれを返す)
  Ref<Image> render_current_frame_main_thread();
};

} // namespace mu
