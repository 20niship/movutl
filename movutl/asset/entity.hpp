#pragma once

#include <cstdint>
#include <cutil/prop.hpp>
#include <cutil/ref.hpp>
#include <cutil/string.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/prop_types.hpp>
#include <mutex>
#include <string>
#include <vector>

#define BITMAPINFOHEADER void
#define WAVEFORMATEX void

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

struct InputPluginTable;
struct FilterPluginTable;
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

  int fstart = -1;                    // MPROPERTY(name="開始位置(frame)" hidden_inspector=true)
  int fend   = -1;                    // MPROPERTY(name="終了位置(frame)" hidden_inspector=true)
  Vec2 anchor;                        // MPROPERTY(name="アンカー", viewer_anchor=true, position=true)
  BlendType blend_     = Blend_Alpha; // MPROPERTY(name="合成モード")
  uint32_t group_guid  = 0;           // MPROPERTY(name="グループID", desc="グループ化されている時のグループID", hidden_inspector=true)
  bool active_         = true;        // MPROPERTY(name="アクティブ", desc="オブジェクトが有効かどうか")
  bool solo_           = false;       // MPROPERTY(name="ソロモード", desc="(音声のみ)他のレイヤを非表示にする")
  bool clipping_up     = false;       // MPROPERTY(name="上レイヤでクリッピング",  hidden_inspector=true)
  bool camera_ctrl     = false;       // MPROPERTY(name="カメラ制御", desc="カメラ制御の対象", hidden_inspector=true)
  int32_t custom_color = 0;           // MPROPERTY(name="カスタム色", desc="0の場合メディア種別ごとの既定色を使う")

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
  std::vector<FilterParam> filters;

  // このEntity固有の状態(トラック情報/img_/デコーダハンドル等)を読み書きする際のロック。Composition::mtxとは別物
  mutable std::mutex mtx;

  virtual constexpr EntityType getType() const = 0;

  static Ref<Entity> CreateEntity(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);

  // プロジェクト保存用: type/name/guid/propsをまとめてシリアライズ/デシリアライズする
  cutil::Prop getSaveProps() const;
  static Ref<Entity> fromSaveProps(const cutil::Prop& p);

  Composition* get_comp() const;
  virtual bool render(Composition* cmp, Image* target, int frame) = 0;

  InputPluginTable* get_input_plugin() const { return in_plg_; }
  InputHandle get_input_handle() const { return in_handle_; }
  const EntityInfo& get_info() const { return info; }

  // 複製後に呼ばれる。素材を持つEntityはoverrideしload_file()等を呼び直し、複製元と独立したプラグインインスタンスを持たせる
  virtual void reload_asset() {}

  bool visible(int frame) const { return fstart <= frame && frame <= fend && active_; }
  virtual ~Entity();

  // 基底のトラック情報(fstart/fend/anchor等)のシリアライズ。派生クラスはoverrideし自身の固有プロパティを別途扱う
  virtual const cutil::PropInfo* getPropsInfo() const; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props);     // MUFUNC_AUTOGEN
};

} // namespace mu
