#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <movutl/paint/brush.hpp>
#include <movutl/raster/tiled_image.hpp>
#include <vector>

namespace mu {

// 選択範囲(8bitマスク)。Strokeにset_clip()すると、全サブクラスが選択の内側だけを書き換える
struct Selection {
  int w = 0, h = 0;
  std::vector<uint8_t> m;
  static Selection rect(int w, int h, int x0, int y0, int x1, int y1); // [x0,x1) x [y0,y1)
  float at(int x, int y) const { return (x < 0 || y < 0 || x >= w || y >= h) ? 0.0f : m[(size_t)y * w + x] / 255.0f; }
};

// 軌跡を spacing 間隔の点列に補間する部品
struct StampPoint {
  float x, y, pressure, angle; // angleは進行方向(ラジアン)
};
class StampPath {
public:
  bool started() const { return has_; }
  const PenSample& last() const { return last_; }
  float angle() const { return angle_; }
  template <class F> void add(const PenSample& p, float step, F&& emit) {
    if(!has_) {
      emit(StampPoint{p.x, p.y, p.pressure, angle_});
      last_  = p;
      has_   = true;
      carry_ = 0;
      return;
    }
    const float dx = p.x - last_.x, dy = p.y - last_.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if(len <= 0.0f) return;
    angle_         = std::atan2(dy, dx);
    float pos      = step - carry_; // 次のスタンプまでの線分上の距離
    float last_pos = -carry_;
    while(pos <= len) {
      const float t = pos / len;
      emit(StampPoint{last_.x + dx * t, last_.y + dy * t, last_.pressure + (p.pressure - last_.pressure) * t, angle_});
      last_pos = pos;
      pos += step;
    }
    carry_ = len - last_pos;
    last_  = p;
  }

private:
  bool has_ = false;
  PenSample last_;
  float carry_ = 0, angle_ = 0;
};

// スタンプ1回分の位置・大きさ・濃さ(筆圧やばらつきを反映済み)
struct StampInfo {
  float x = 0, y = 0, radius = 0, angle = 0;
  float flow     = 1; // 濃さ(opacity x 筆圧 x 呼び出し側の倍率)
  float pressure = 1;
};
// スタンプ1回分の被覆率(形 x 紙目 x 選択範囲)。flowは含まない
struct Stamp {
  int x0 = 0, y0 = 0, w = 0, h = 0;
  std::vector<float> coverage;
};

// ペン/マウスの動きから画像を書き換える共通オブジェクト。継承して任意の描画方式を作れる
// (消しゴム・テクスチャ貼り・トーン・パーティクル・水彩・エアブラシ等)。add()の中で何をしてもよい
class Stroke {
public:
  Stroke(TiledImage* target, const BrushSettings& s);
  virtual ~Stroke()                    = default;
  virtual void add(const PenSample& p) = 0; // ペン/マウスの1サンプル
  virtual void tick(float dt) { (void)dt; } // ペンを下ろしている間の時間経過(エアブラシ等)
  virtual void end() {}                     // ペンを上げた(水彩の確定等)
  void set_clip(const Selection* s) { clip_ = s; }

protected:
  // 以下は部品。使わなくてもよい
  StampInfo stamp_info(const PenSample& s, float dir_angle, float flow_mul = 1.0f); // 筆圧・ばらつきを反映
  bool make_stamp(const StampInfo& info, Stamp& out);                               // 空ならfalse
  template <class F> void stamp_along(const PenSample& p, F&& f) {                  // 軌跡に沿って f(StampInfo) を呼ぶ
    path_.add(p, std::max(0.5f, settings_.spacing * settings_.size), [&](const StampPoint& sp) { f(stamp_info({sp.x, sp.y, sp.pressure}, sp.angle)); });
  }
  float rnd01(); // ストロークごとに決定的な乱数

  TiledImage* target_;
  BrushSettings settings_;
  const Selection* clip_ = nullptr;
  std::shared_ptr<const BrushTip> tip_;
  StampPath path_;

private:
  uint32_t rng_;
};

} // namespace mu
