#pragma once

#include <cstdint>
#include <cutil/prop.hpp>
#include <memory>
#include <movutl/core/vector.hpp>
#include <movutl/raster/pixel_buffer.hpp>
#include <string>

namespace mu {

// ペン入力1サンプル。座標はキャンバス座標(px)、pressureは0〜1(マウスは1固定)
struct PenSample {
  float x = 0, y = 0;
  float pressure = 1.0f;
};

// ---- ブラシ先端(形) ----
// スタンプ1回につき1回だけ呼ばれる(ピクセルごとの仮想呼び出しにはならない)。派生クラスで任意の形を作れる
struct TipParams {
  float radius = 5;     // 半径(px)
  float angle  = 0;     // 回転(ラジアン、時計回りが正)
  float cx = 0, cy = 0; // スタンプ中心(maskローカルの連続座標。ピクセル(i,j)の中心は(i+0.5,j+0.5))
};
class BrushTip {
public:
  virtual ~BrushTip() = default;
  // w x hのmask(0〜1の被覆率、行優先、0で初期化済み)へ形を書く
  virtual void render(const TipParams& p, int w, int h, float* mask) const = 0;
};

// 円形。hardnessが1で縁までべた塗り、0で中心から縁へ線形に薄くなる
class CircleTip : public BrushTip {
public:
  explicit CircleTip(float hardness = 0.8f) : hardness_(hardness) {}
  void render(const TipParams& p, int w, int h, float* mask) const override;

private:
  float hardness_;
};

// 画像先端(Photoshopのブラシ先端と同じ規約: 黒=描く、白=描かない、アルファも掛ける)。画像は先端の直径に正規化して貼る
class ImageTip : public BrushTip {
public:
  explicit ImageTip(std::shared_ptr<const PixelBuffer8> img) : img_(std::move(img)) {}
  void render(const TipParams& p, int w, int h, float* mask) const override;

private:
  std::shared_ptr<const PixelBuffer8> img_;
};

// 画像ファイル(png/jpg等)を読む。失敗時はnullptr。ブラシ先端・テクスチャ・パターン画像の読み込みに使う(assets/brushes/)
std::shared_ptr<PixelBuffer8> load_brush_image(const std::string& path);

// ---- 紙目テクスチャ ----
// キャンバス座標に固定され(ストロークが動いても紙目は動かない)、被覆率に掛ける。濃い所ほどよく乗り、薄い所は乗らない
struct BrushTexture {
  std::shared_ptr<const PixelBuffer8> img; // 輝度を使う(白=よく乗る)。nullなら無効
  float scale = 1.0f;                      // 画像1pxがキャンバス何pxか
  float depth = 1.0f;                      // 0で無効、1で最大
  float sample(int x, int y) const;        // 掛ける係数(0〜1)
};

struct BrushSettings {
  std::string engine = "paint"; // StrokeRegistryに登録された描画方式の名前

  float size            = 10.0f; // 直径(px)
  float hardness        = 0.8f;  // tip未指定時の円形先端の硬さ
  float opacity         = 1.0f;  // 0〜1
  float spacing         = 0.1f;  // スタンプ間隔(サイズ比)
  bool pressure_size    = true;
  bool pressure_opacity = false;
  Vec4b color           = Vec4b{0, 0, 0, 255};

  std::shared_ptr<const BrushTip> tip; // 先端。nullならtip_name、それも空なら円形(hardness)
  std::string tip_name;                // TipRegistryに登録された先端の名前
  BrushTexture texture;

  float angle       = 0;     // 先端の回転(ラジアン)
  bool angle_follow = false; // 進行方向に先端を向ける
  float size_jitter = 0;     // 0〜1。スタンプごとにサイズを最大この割合だけ縮める
  float scatter     = 0;     // スタンプ位置のばらつき(サイズ比)
  uint32_t seed     = 1;     // ばらつきの乱数種(ストロークごとに同じ結果になる)

  float airbrush_rate = 2.0f; // "airbrush": 止まっている間に1秒あたり積む濃さ(opacityの倍数)

  std::shared_ptr<const PixelBuffer8> pattern; // "pattern": 色の代わりに貼る画像
  float pattern_scale = 1.0f;

  float tone_pitch = 6.0f;   // "tone": 網点の間隔(px)
  float tone_angle = 0.785f; // "tone": 網点の角度(ラジアン)

  int particle_count  = 8;    // "particle": 1スタンプあたりの粒数
  float particle_size = 0.2f; // "particle": 粒の大きさ(スタンプ半径比)

  float water_mix      = 0.5f; // "watercolor": 下地の色との混ざりやすさ(0〜1)
  float edge_darkening = 0.5f; // "watercolor": 縁の濃さ(0〜1)

  cutil::Prop params; // プラグインのブラシ固有パラメータ(StrokePluginTable::fn_initで宣言)
};

} // namespace mu
