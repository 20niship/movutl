#pragma once

#include <cmath>
#include <doctest/doctest.h>
#include <movutl/paint/paint_document.hpp>
#include <vector>

namespace mu::test {

// テスト用の仮想ペン。実際のペンタブの代わりに、決まった軌跡・筆圧・時間経過をPaintDocumentへ流し込む
struct VirtualPen {
  float dt = 1.0f / 120.0f; // サンプル間の時間(秒)。tick()で渡される

  using Pressure = float (*)(float t); // t=0〜1(軌跡上の位置)→筆圧
  static float full(float) { return 1.0f; }
  static float bell(float t) { return std::sin(3.14159265f * t); } // 入り抜き(筆圧が山形)
  static float ramp_up(float t) { return 0.05f + 0.95f * t; }      // 弱→強
  static float ramp_down(float t) { return 1.0f - 0.95f * t; }

  // x0→x1を、振幅amp・周期cyclesのサイン波で進む軌跡
  static std::vector<PenSample> wave(float x0, float x1, float y, float amp, float cycles, int n, Pressure p = full) {
    std::vector<PenSample> v;
    for(int i = 0; i <= n; i++) {
      const float t = (float)i / n;
      v.push_back({x0 + (x1 - x0) * t, y + amp * std::sin(t * cycles * 6.2831853f), p(t)});
    }
    return v;
  }
  static std::vector<PenSample> line(float x0, float y0, float x1, float y1, int n, Pressure p = full) {
    std::vector<PenSample> v;
    for(int i = 0; i <= n; i++) {
      const float t = (float)i / n;
      v.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, p(t)});
    }
    return v;
  }
  static std::vector<PenSample> spiral(float cx, float cy, float r0, float r1, float turns, int n, Pressure p = full) {
    std::vector<PenSample> v;
    for(int i = 0; i <= n; i++) {
      const float t = (float)i / n, a = t * turns * 6.2831853f, r = r0 + (r1 - r0) * t;
      v.push_back({cx + r * std::cos(a), cy + r * std::sin(a), p(t)});
    }
    return v;
  }

  // ペンを下ろし、pathを順に動かし(毎サンプルtick)、最後にhold_sec秒止めてから上げる
  void draw(PaintDocument& d, int layer, const BrushSettings& s, const std::vector<PenSample>& path, float hold_sec = 0.0f) const {
    REQUIRE(d.begin_stroke(layer, s));
    for(const auto& p : path) {
      d.add_sample(p);
      d.tick(dt);
    }
    for(float t = 0; t < hold_sec; t += dt) d.tick(dt);
    d.end_stroke();
  }
};

} // namespace mu::test
