#pragma once

#include <algorithm>
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

struct InputPluginTable;
struct FilterPluginTable;
class Composition;
class Image;

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
  EntityType_Midi        = 1 << 16, // VST音源で再生するMIDIノート列(MidiEntt)
  EntityType_SceneChange = 1 << 17, // AviUtlのシーンチェンジ(SceneChangeEntt)
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

// 描画変換(座標は中心原点)。グループ制御の親変換としても使う
struct GroupXform {
  Vec3 pos       = Vec3(0, 0, 0);
  float scale    = 100.0f; // %
  float rotation = 0.0f;
  float alpha    = 1.0f;

  // this(親)を適用した後にchild(子の局所変換)を置いた合成変換。結合的なので親を外側から順に畳み込める
  GroupXform compose(const GroupXform& child) const;
};

// Entity::render()の間だけ、そのスレッドで描画するEntityへ親グループの合成変換を与える(composite()が参照する)
class GroupXformScope {
  const GroupXform* prev_;

public:
  explicit GroupXformScope(const GroupXform* parent);
  ~GroupXformScope();
  GroupXformScope(const GroupXformScope&)            = delete;
  GroupXformScope& operator=(const GroupXformScope&) = delete;
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
  int fstart_           = -1;          // MPROPERTY(name="開始位置(frame)", hidden_inspector=true, group="track")
  int fend_             = -1;          // MPROPERTY(name="終了位置(frame)", hidden_inspector=true, group="track")
  BlendType blend_      = Blend_Alpha; // MPROPERTY(name="合成モード", group="track")
  uint32_t group_guid_  = 0;           // MPROPERTY(name="グループID", desc="グループ化されている時のグループID", hidden_inspector=true, group="track")
  bool active_          = true;        // MPROPERTY(name="アクティブ", desc="オブジェクトが有効かどうか", group="track")
  bool solo_            = false;       // MPROPERTY(name="ソロモード", desc="(音声のみ)他のレイヤを非表示にする", group="track")
  bool clipping_up_     = false;       // MPROPERTY(name="上レイヤでクリッピング", desc="1つ上のオブジェクトの形で切り抜く", group="track")
  bool camera_ctrl_     = false;       // MPROPERTY(name="カメラ制御", desc="カメラ制御の対象", group="track")
  int32_t custom_color_ = 0;           // MPROPERTY(name="カスタム色", desc="0の場合メディア種別ごとの既定色を使う", group="track")
  std::vector<FilterParam> filters_;

  // 描画系Entity共通の変換。座標はコンポジション中心原点・Y下向き、単位はpx/%/度(時計回りが正)/0-1。
  // pos_はanchor_(画像中心からの基点オフセット)が置かれる位置で、回転・拡大は基点まわりに行う。
  Vec3 pos_       = Vec3(0, 0, 0); // MPROPERTY(name="位置(px)", viewer_anchor=true, position=true, group="transform")
  Vec3 anchor_    = Vec3(0, 0, 0); // MPROPERTY(name="基点(px)", desc="画像中心からの基点オフセット。回転・拡大の中心", group="transform")
  float scale_    = 100.0f;        // MPROPERTY(name="拡大率(%)", min=0.0, step=1.0, group="transform")
  float rotation_ = 0.0f;          // MPROPERTY(name="回転(度)", angle=true, group="transform")
  float rot_x_    = 0.0f;          // MPROPERTY(name="X軸回転(度)", angle=true, group="transform")
  float rot_y_    = 0.0f;          // MPROPERTY(name="Y軸回転(度)", angle=true, group="transform")
  float aspect_   = 0.0f;          // MPROPERTY(name="縦横比", desc="-1〜1。正で横が縮み(縦長)、負で縦が縮む(横長)", min=-1.0, max=1.0, step=0.01, group="transform")
  float alpha_    = 1.0f;          // MPROPERTY(name="不透明度(%)", min=0.0, max=1.0, step=0.01, group="transform")

  // getPropsInfo()を持つEntityの中間点(キーフレーム)アニメーション。ensure_anim_props()でgetProps()から遅延構築される
  mutable AnimProps anim_props_;

  // このEntity固有の状態(img_/デコーダハンドル等)を読み書きする際のロック。Composition::mtxとは別物
  mutable std::mutex mtx;

  // 自身の局所変換にGroupXformScopeで与えられた親グループ変換を合成した、実際に描画に使う変換
  GroupXform world_xform() const;

  // srcを自身の変換(pos_/anchor_/scale_/aspect_/rotation_/rot_x_/rot_y_/alpha_/blend_)でtargetへ合成する。rot_x_/rot_y_が0でなければ射影変換(2D合成のみ、Zは奥行きの遠近のみ)
  // origin_offset: srcの中心から見た、このEntityの局所原点(基点の既定位置)のずれ(px)。通常は0(=画像中心)
  bool composite(const Image& src, Image* target, const Vec2& origin_offset = Vec2(0, 0)) const;

  virtual constexpr EntityType getType() const = 0;

  // pos_/anchor_/scale_/rotation_/alpha_による描画変換を持つ(=描画系の)Entityか。インスペクタの変換欄表示に使う
  bool has_transform() const { return getType() & (EntityType_Movie | EntityType_Image | EntityType_3DText | EntityType_Polygon | EntityType_Framebuffer | EntityType_Scene | EntityType_Group | EntityType_Camera); }

  // composite()へ渡す元画像のサイズと基点の既定位置のずれ(画像中心基準)。ビューアのギズモが枠を求めるのに使う。画像が無い/不明ならfalse
  virtual bool source_size(Vec2& size, Vec2& origin_offset) const {
    (void)size;
    (void)origin_offset;
    return false;
  }

  static Ref<Entity> CreateEntity(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);

  // プロジェクト保存用: type/name/guid/props/トラック共通属性をまとめてシリアライズ/デシリアライズする
  cutil::Prop getSaveProps() const;
  static Ref<Entity> fromSaveProps(const cutil::Prop& p);

  Composition* get_comp() const;
  virtual bool render(Composition* cmp, Image* target, int frame) = 0;
  virtual bool fetch_audio(Composition*, int64_t, int, int16_t*) { return false; }

  InputPluginTable* get_input_plugin() const { return in_plg_; }
  InputHandle get_input_handle() const { return in_handle_; }
  const EntityInfo& get_info() const { return info; }

  // 複製後に呼ばれる。素材を持つEntityはoverrideしload_file()等を呼び直し、複製元と独立したプラグインインスタンスを持たせる
  virtual void reload_asset() {}

  bool visible(int frame) const { return fstart_ <= frame && frame <= fend_ && active_; }
  virtual ~Entity();

  // 旧TrackObject::getPropsInfo/getProps/setPropsの移行先。上記のトラック共通属性のみを対象とする
  // (pygenの出力順が環境で変わらないよう、名前順(Track→Transform)で宣言する)
  const cutil::PropInfo* getTrackPropsInfo() const; // MUFUNC_AUTOGEN
  cutil::Prop getTrackProps() const;                // MUFUNC_AUTOGEN
  void setTrackProps(const cutil::Prop& props);     // MUFUNC_AUTOGEN

  const cutil::PropInfo* getTransformPropsInfo() const; // MUFUNC_AUTOGEN
  cutil::Prop getTransformProps() const;                // MUFUNC_AUTOGEN
  void setTransformProps(const cutil::Prop& props);     // MUFUNC_AUTOGEN

  virtual const cutil::PropInfo* getPropsInfo() const { return nullptr; }
  virtual cutil::Prop getProps() const { return {}; }
  virtual void setProps(const cutil::Prop& props) { (void)props; }

  // レンダリング直前に呼び、anim_props_をframe時点の値へ評価してsetProps()へ反映する(getPropsInfo()を持たないEntityは何もしない)
  void apply_animated_props(int frame);

  // メンバ(pos_/anchor_/scale_/rotation_)の現在値をanim_props_のframe時点へ書き戻す。ギズモ等がメンバを直接書き換えた時、次のapply_animated_props()で古い値に戻されるのを防ぐ
  void store_xform_to_anim(int frame);

  // トラックの長さ(fstart_/fend_)を変更した/splitした直後に呼ぶ。old_startは変更前のfstart_。
  // 中間点は開始からの相対frameなので、開始が動いた分だけキーを逆方向へ動かし、範囲(0〜fend_-fstart_)外のキーは境界の補間値キーに置き換える
  void on_len_change_done(int old_start);

  // コンポジション絶対frameをトラック開始からの相対frame(中間点のキー)へ変換する。開始より前は0
  uint32_t rel_frame(int abs_frame) const { return (uint32_t)std::max(abs_frame - std::max(fstart_, 0), 0); }

  // 未初期化(size()==0)ならgetProps()から構築する(派生クラスは独自Create()で直接constructしCreateEntity()を経由しないため遅延初期化にする)
  void ensure_anim_props() const;

  // タイムライン集約表示用: anim_props_ + 全filters_[].propsの全キーフレームを(コンポジション絶対frameへ直して)重複排除・昇順でまとめる
  std::vector<uint32_t> collect_animated_frames() const;
  // (コンポジション絶対frame指定)old_frameにあるキーフレームを全プロパティ横断でnew_frameへ一括移動する。1つでも動けばtrue
  bool move_keyframes_at(uint32_t old_frame, uint32_t new_frame);
  // (コンポジション絶対frame指定)frameにあるキーフレームを全プロパティ横断で一括削除する。1つでも消せればtrue
  bool erase_keyframes_at(uint32_t frame);
};

} // namespace mu
