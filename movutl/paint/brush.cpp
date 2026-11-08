#include <algorithm>
#include <cmath>
#include <movutl/paint/brush.hpp>
#include <movutl/raster/blend.hpp>

namespace mu {

void BrushStroke::add(const PenSample& p) {
  if(!has_last_) {
    stamp(p);
    last_     = p;
    has_last_ = true;
    carry_    = 0;
    return;
  }
  const float dx = p.x - last_.x, dy = p.y - last_.y;
  const float len = std::sqrt(dx * dx + dy * dy);
  if(len <= 0.0f) return;
  const float step = std::max(0.5f, s_.spacing * s_.size);
  float pos        = step - carry_; // 次のスタンプまでの線分上の距離
  float last_pos   = -carry_;
  while(pos <= len) {
    const float t = pos / len;
    stamp({last_.x + dx * t, last_.y + dy * t, last_.pressure + (p.pressure - last_.pressure) * t});
    last_pos = pos;
    pos += step;
  }
  carry_ = len - last_pos;
  last_  = p;
}

void BrushStroke::stamp(const PenSample& p) {
  const float pr     = std::clamp(p.pressure, 0.0f, 1.0f);
  const float radius = 0.5f * s_.size * (s_.pressure_size ? std::max(pr, 0.05f) : 1.0f);
  const float flow   = s_.opacity * (s_.pressure_opacity ? pr : 1.0f);
  if(radius <= 0.0f || flow <= 0.0f) return;
  const float inner = radius * std::clamp(s_.hardness, 0.0f, 1.0f);
  const float edge  = std::max(radius - inner, 1.0f); // 最低1pxはぼかしてアンチエイリアスにする
  const int x0 = std::max(0, (int)std::floor(p.x - radius)), x1 = std::min(img_->width() - 1, (int)std::ceil(p.x + radius));
  const int y0 = std::max(0, (int)std::floor(p.y - radius)), y1 = std::min(img_->height() - 1, (int)std::ceil(p.y + radius));
  for(int y = y0; y <= y1; y++) {
    for(int x = x0; x <= x1; x++) {
      const float d = std::hypot(x + 0.5f - p.x, y + 0.5f - p.y);
      if(d >= radius) continue;
      const float cov = std::clamp((radius - d) / edge, 0.0f, 1.0f) * flow;
      if(cov <= 0.0f) continue;
      Vec4b& px = img_->at(x, y);
      if(s_.eraser)
        px[3] = (unsigned char)std::clamp(px[3] * (1.0f - cov) + 0.5f, 0.0f, 255.0f);
      else
        blend_over_straight(px, s_.color, cov * s_.color[3] / 255.0f);
    }
  }
}

} // namespace mu
