#pragma once

#include <atomic>
#include <cutil/prop.hpp>
#include <cutil/string.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/render2d/frame_cache.hpp>
#include <mutex>

namespace mu {

class Entity;
class AudioRingBuffer;

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
  // 現在の表示フレーム。ワーカースレッドとGUIスレッドの双方から高頻度に読み書きされるためmtx無しで安全にアクセスできるようatomicにしている
  std::atomic<int32_t> frame{0};

  // ---------- audio ----------
  int32_t audio_sample_rate = 48000; // 音声サンプリングレート(Hz)
  int32_t audio_channels    = 2;     // 音声チャンネル数
  Ref<AudioRingBuffer> audio_buf; // AudioMixWorkerが書き込むミックス済みPCM。内部mutexで保護(mtxとは別ロック)

  // frameとaudio_sample_rateからサンプル位置へ変換する
  int64_t frame_to_sample(int frame) const { return (int64_t)((double)frame / (double)framerate * (double)audio_sample_rate); }

  // ---------- track ----------
  std::vector<TrackLayer> layers;

  // layers(レイヤー構成/エンティティの追加削除)を読み書きする際のロック。各Entity固有の状態はEntity::mtxで個別に守る
  mutable std::mutex mtx;
  // フレーム単位のレンダリング結果キャッシュ(バックグラウンドレンダーワーカーが書き込む)
  FrameCache cache;

  void resize(int32_t w, int32_t h);

  Composition();
  Composition(const char* name, int32_t w = 1920, int32_t h = 1080, int32_t fps = 30);
  std::string str() const;
  std::string summary() const;

  static Composition* GetActiveComp();

  const cutil::PropInfo* getPropsInfo() const;
  cutil::Prop getProps() const;
  void setProps(const cutil::Prop& props);

  int insertable_layer_index() const;
  void insert_entity(Ref<Entity> entt, int layer = -1);

  // アクティブなレイヤーに乗っている全Entityのスナップショットを返す(mtxを短時間だけlockする)
  std::vector<Ref<Entity>> get_all_entities() const;

  // 現在フレームをバックグラウンドキューを使わずその場で同期レンダリングして取得する(キャッシュ済みならそれを返す)
  Ref<Image> render_current_frame_main_thread();

  // frameはstd::atomicのためLuaバインディング等から扱うにはこの2関数を使う
  int32_t get_frame() const { return frame.load(); }
  void set_frame(int32_t f) { frame.store(f); }
};

} // namespace mu
