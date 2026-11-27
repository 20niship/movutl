#pragma once

#include <algorithm>
#include <cmath>
#include <movutl/core/vector.hpp>

namespace mu {

// キャンバスの表示位置(移動・拡大縮小・回転・左右反転)と座標変換。ImGuiに依存しない
// screen = ビュー中心 + pan + 回転(angle) * 反転(flip) * zoom * (canvas - キャンバス中心)
struct CanvasView {
  static constexpr float kMinZoom = 0.01f, kMaxZoom = 64.0f;

  Vec2 view_pos, view_size; // 表示領域(screen座標)。毎フレーム設定する
  Vec2 canvas_size;         // キャンバスの大きさ(px)
  Vec2 pan;                 // 表示中心のずれ(screen px)
  float zoom  = 1.0f;
  float angle = 0.0f; // ラジアン、時計回りが正
  bool flip   = false;

  Vec2 to_screen(const Vec2& c) const {
    const float x = (c[0] - canvas_size[0] * 0.5f) * (flip ? -zoom : zoom), y = (c[1] - canvas_size[1] * 0.5f) * zoom;
    const float ca = std::cos(angle), sa = std::sin(angle);
    return Vec2(center()[0] + pan[0] + x * ca - y * sa, center()[1] + pan[1] + x * sa + y * ca);
  }
  Vec2 to_canvas(const Vec2& s) const {
    const float x = s[0] - center()[0] - pan[0], y = s[1] - center()[1] - pan[1];
    const float ca = std::cos(angle), sa = std::sin(angle);
    return Vec2((x * ca + y * sa) / (flip ? -zoom : zoom) + canvas_size[0] * 0.5f, (-x * sa + y * ca) / zoom + canvas_size[1] * 0.5f);
  }

  void move(const Vec2& screen_delta) { pan = Vec2(pan[0] + screen_delta[0], pan[1] + screen_delta[1]); }
  // pivot(screen座標)の下にある点を動かさずに拡大縮小/回転する
  void zoom_at(const Vec2& pivot, float factor) {
    keep(pivot, [&] { zoom = std::clamp(zoom * factor, kMinZoom, kMaxZoom); });
  }
  void rotate_at(const Vec2& pivot, float delta) {
    keep(pivot, [&] { angle = std::remainder(angle + delta, 6.2831853f); });
  }
  // 画面上の見た目を、ビュー中心の縦線で左右反転する
  void flip_horizontal() {
    flip   = !flip;
    angle  = -angle;
    pan[0] = -pan[0];
  }
  // 回転と反転を保ったまま、全体が収まる大きさで中央に表示する
  void fit() {
    const float ca = std::abs(std::cos(angle)), sa = std::abs(std::sin(angle));
    const float w = canvas_size[0] * ca + canvas_size[1] * sa, h = canvas_size[0] * sa + canvas_size[1] * ca;
    pan = Vec2(0, 0);
    if(w > 0 && h > 0) zoom = std::clamp(std::min(view_size[0] / w, view_size[1] / h), kMinZoom, kMaxZoom);
  }

private:
  Vec2 center() const { return Vec2(view_pos[0] + view_size[0] * 0.5f, view_pos[1] + view_size[1] * 0.5f); }
  template <class F> void keep(const Vec2& pivot, F&& change) {
    const Vec2 c = to_canvas(pivot);
    change();
    const Vec2 s = to_screen(c);
    move(Vec2(pivot[0] - s[0], pivot[1] - s[1]));
  }
};

} // namespace mu
