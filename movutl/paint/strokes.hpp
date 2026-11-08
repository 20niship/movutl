#pragma once

#include <movutl/paint/stroke.hpp>

namespace mu {

// 標準の描画方式。いずれもStrokeを継承した実装例でもある

// 色をover合成する(通常のブラシ)
class PaintStroke : public Stroke {
public:
  using Stroke::Stroke;
  void add(const PenSample& p) override {
    stamp_along(p, [&](const StampInfo& s) { paint(s); });
  }

protected:
  void paint(const StampInfo& s);
};

// 消しゴム: アルファを減らす
class EraseStroke : public Stroke {
public:
  using Stroke::Stroke;
  void add(const PenSample& p) override;
};

// エアブラシ: ペンを止めていてもtick()で同じ場所に濃さを積む
class AirbrushStroke : public PaintStroke {
public:
  using PaintStroke::PaintStroke;
  void tick(float dt) override {
    if(path_.started()) paint(stamp_info(path_.last(), path_.angle(), settings_.airbrush_rate * dt));
  }
};

// テクスチャ貼り: 色の代わりにpattern画像をキャンバス座標で貼る
class PatternStroke : public Stroke {
public:
  using Stroke::Stroke;
  void add(const PenSample& p) override;
};

// トーン貼り: 濃さを網点で表現する(2値)
class ToneStroke : public Stroke {
public:
  using Stroke::Stroke;
  void add(const PenSample& p) override;
};

// パーティクル: 1スタンプから小さな粒を撒く
class ParticleStroke : public PaintStroke {
public:
  using PaintStroke::PaintStroke;
  void add(const PenSample& p) override;
};

// 水彩: 下地の色と混ざり、縁が濃くなる。ストローク中は作業バッファに溜め、end()で確定する
class WatercolorStroke : public Stroke {
public:
  WatercolorStroke(TiledImage* target, const BrushSettings& s) : Stroke(target, s), wet_(target->width(), target->height()) {}
  void add(const PenSample& p) override;
  void end() override;

private:
  TiledImage wet_; // アルファに顔料の濃さを溜める
};

} // namespace mu
