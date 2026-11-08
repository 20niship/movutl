#pragma once

#include <movutl/core/vector.hpp>
#include <movutl/raster/tiled_image.hpp>

namespace mu {

// ペン入力1サンプル。座標はキャンバス座標(px)、pressureは0〜1(マウスは1固定)
struct PenSample {
  float x = 0, y = 0;
  float pressure = 1.0f;
};

struct BrushSettings {
  float size            = 10.0f; // 直径(px)
  float hardness        = 0.8f;  // 0〜1。1で縁までべた塗り、0で中心から縁へ線形に薄くなる
  float opacity         = 1.0f;  // 0〜1
  float spacing         = 0.1f;  // スタンプ間隔(サイズ比)
  bool pressure_size    = true;  // 筆圧で太さを変える
  bool pressure_opacity = false; // 筆圧で不透明度を変える
  bool eraser           = false; // trueなら描画せずアルファを減らす
  Vec4b color           = Vec4b{0, 0, 0, 255};
};

// 1ストローク分の描画。サンプルを順にadd()すると、前のサンプルとの間をspacing間隔の円スタンプで埋める
class BrushStroke {
public:
  BrushStroke(TiledImage* img, const BrushSettings& s) : img_(img), s_(s) {}
  void add(const PenSample& p);

private:
  void stamp(const PenSample& p);
  TiledImage* img_;
  BrushSettings s_;
  bool has_last_ = false;
  PenSample last_;
  float carry_ = 0; // 最後のスタンプから現在位置までの距離
};

} // namespace mu
