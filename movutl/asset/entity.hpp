#pragma once

#include <cutil/string.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/prop_types.hpp>
#include <mutex>
#include <string>

#define BITMAPINFOHEADER void
#define WAVEFORMATEX void

namespace mu {

struct InputPluginTable;
class Composition;
class Image;

enum EntityType {
  EntityType_Movie       = 1,
  EntityType_Audio       = 1 << 1,
  EntityType_Image       = 1 << 2,
  EntityType_3DText      = 1 << 3,
  EntityType_Primitive   = 1 << 4,
  EntityType_Framebuffer = 1 << 5,
  EntityType_Polygon     = 1 << 6,
  EntityType_Group       = 1 << 7,
  EntityType_Scene       = 1 << 8,
  EntityType_SceneAudio  = 1 << 9,
  EntityType_LayerCopy   = 1 << 10,
  EntityType_Particle    = 1 << 11,
  EntityType_Custom      = 1 << 12,
  EntityType_3DModel     = 1 << 13,
  EntityType_Camera      = 1 << 14,
  EntityType_Effect      = 1 << 15,
};
MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS(EntityType);

// EntityType_Polygon(ShapeEntt)が描画する図形の種類
enum ShapeType {
  ShapeType_Triangle = 0,
  ShapeType_Rect     = 1,
  ShapeType_Hexagon  = 2,
  ShapeType_Circle   = 3,
  ShapeType_Custom   = 4, // custom_pathで指定した任意の多角形
};

// プラグインがそのファイルを開いた時のインスタンスを返すときのポインタ
typedef void* InputHandle;

enum ImageFormat {
  ImageFormatRGB       = 0,
  ImageFormatRGBA      = 2,
  ImageFormatGRAYSCALE = 1,
};
struct EntityInfo {
  EntityType flag            = EntityType_Movie; // 読み込み可能なオブジェクトの種類
  float framerate            = 30;               // フレームレート
  uint32_t nframes           = 0;                // フレーム数
  ImageFormat format         = ImageFormatRGB;   // 画像フォーマット
  uint16_t width             = 0;                // 画像サイズ
  uint16_t height            = 0;                // 画像サイズ
  int32_t audio_n            = 0;                // 音声サンプル数
  int32_t audio_sample_rate  = 0;                // 音声サンプリングレート(Hz)
  int32_t audio_channels     = 0;                // 音声チャンネル数
  WAVEFORMATEX* audio_format = nullptr;          // 音声フォーマットへのポインタ(次に関数が呼ばれるまで内容を有効にしておく)
  int32_t audio_format_size;                     // 音声フォーマットのサイズ
  void* handler;                                 // 画像codecハンドラ
  int32_t reserved[7];
  std::string str() const;
};

class Entity {
protected:
  InputPluginTable* in_plg_ = nullptr;
  InputHandle in_handle_    = nullptr;
  EntityInfo info;

  bool render_filters(Composition* cmp, Image* img, int frame);

public:
  cutil::Str name;    // MPROPERTY(name="名前")
  uint64_t guid_ = 0; // MPROPERTY(name="GUID")

  // 旧TrackObjectのメンバ。MPROPERTYのgroup="track"はgetTrackPropsInfo()対象を絞り込むpygen用タグ
  int fstart_ = -1;                    // MPROPERTY(name="開始位置(frame)", hidden_inspector=true, group="track")
  int fend_   = -1;                    // MPROPERTY(name="終了位置(frame)", hidden_inspector=true, group="track")
  Vec2 anchor_;                        // MPROPERTY(name="アンカー", viewer_anchor=true, position=true, group="track")
  BlendType blend_      = Blend_Alpha; // MPROPERTY(name="合成モード", group="track")
  uint32_t group_guid_  = 0;           // MPROPERTY(name="グループID", desc="グループ化されている時のグループID", hidden_inspector=true, group="track")
  bool active_          = true;        // MPROPERTY(name="アクティブ", desc="オブジェクトが有効かどうか", group="track")
  bool solo_            = false;       // MPROPERTY(name="ソロモード", desc="(音声のみ)他のレイヤを非表示にする", group="track")
  bool clipping_up_     = false;       // MPROPERTY(name="上レイヤでクリッピング",  hidden_inspector=true, group="track")
  bool camera_ctrl_     = false;       // MPROPERTY(name="カメラ制御", desc="カメラ制御の対象", hidden_inspector=true, group="track")
  int32_t custom_color_ = 0;           // MPROPERTY(name="カスタム色", desc="0の場合メディア種別ごとの既定色を使う", group="track")
  std::vector<FilterParam> filters_;

  // このEntity固有の状態(img_/デコーダハンドル等)を読み書きする際のロック。Composition::mtxとは別物
  mutable std::mutex mtx;

  virtual constexpr EntityType getType() const = 0;

  static Ref<Entity> CreateEntity(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);

  // プロジェクト保存用: type/name/guid/props/トラック共通属性をまとめてシリアライズ/デシリアライズする
  cutil::Prop getSaveProps() const;
  static Ref<Entity> fromSaveProps(const cutil::Prop& p);

  Composition* get_comp() const;
  virtual bool render(Composition* cmp, Image* target, int frame) = 0;

  InputPluginTable* get_input_plugin() const { return in_plg_; }
  InputHandle get_input_handle() const { return in_handle_; }
  const EntityInfo& get_info() const { return info; }

  // 複製後に呼ばれる。素材を持つEntityはoverrideしload_file()等を呼び直し、複製元と独立したプラグインインスタンスを持たせる
  virtual void reload_asset() {}

  bool visible(int frame) const { return fstart_ <= frame && frame <= fend_ && active_; }
  virtual ~Entity();

  // 旧TrackObject::getPropsInfo/getProps/setPropsの移行先。上記のトラック共通属性のみを対象とする
  const cutil::PropInfo* getTrackPropsInfo() const; // MUFUNC_AUTOGEN
  cutil::Prop getTrackProps() const;                // MUFUNC_AUTOGEN
  void setTrackProps(const cutil::Prop& props);     // MUFUNC_AUTOGEN

  virtual const cutil::PropInfo* getPropsInfo() const { return nullptr; }
  virtual cutil::Prop getProps() const { return {}; }
  virtual void setProps(const cutil::Prop& props) { (void)props; }
};

} // namespace mu
