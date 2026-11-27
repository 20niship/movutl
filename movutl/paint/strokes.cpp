#include <movutl/paint/strokes.hpp>
#include <movutl/raster/blend.hpp>

namespace mu {

void PaintStroke::paint(const StampInfo& s) {
  Stamp st;
  if(!make_stamp(s, st)) return;
  const float ca = settings_.color[3] / 255.0f;
  for(int j = 0; j < st.h; j++) {
    for(int i = 0; i < st.w; i++) {
      const float cov = st.coverage[(size_t)j * st.w + i] * s.flow;
      if(cov > 0.0f) blend_over_straight(target_->at(st.x0 + i, st.y0 + j), settings_.color, cov * ca);
    }
  }
}

void EraseStroke::add(const PenSample& p) {
  stamp_along(p, [&](const StampInfo& s) {
    Stamp st;
    if(!make_stamp(s, st)) return;
    for(int j = 0; j < st.h; j++) {
      for(int i = 0; i < st.w; i++) {
        const float cov = std::min(1.0f, st.coverage[(size_t)j * st.w + i] * s.flow);
        if(cov <= 0.0f) continue;
        Vec4b& d = target_->at(st.x0 + i, st.y0 + j);
        d[3]     = (unsigned char)std::clamp(d[3] * (1.0f - cov) + 0.5f, 0.0f, 255.0f);
      }
    }
  });
}

void PatternStroke::add(const PenSample& p) {
  const auto& pat = settings_.pattern;
  if(!pat || pat->empty()) return;
  const float sc = std::max(settings_.pattern_scale, 0.01f);
  auto wrap      = [](float v, int n) {
    int i = (int)std::floor(v) % n;
    return i < 0 ? i + n : i;
  };
  stamp_along(p, [&](const StampInfo& s) {
    Stamp st;
    if(!make_stamp(s, st)) return;
    for(int j = 0; j < st.h; j++) {
      for(int i = 0; i < st.w; i++) {
        const float cov = st.coverage[(size_t)j * st.w + i] * s.flow;
        if(cov <= 0.0f) continue;
        const int x = st.x0 + i, y = st.y0 + j;
        const Vec4b& c = (*pat)(wrap(x / sc, pat->width()), wrap(y / sc, pat->height())); // キャンバス座標に固定
        blend_over_straight(target_->at(x, y), c, cov * c[3] / 255.0f);
      }
    }
  });
}

void ToneStroke::add(const PenSample& p) {
  const float pitch = std::max(settings_.tone_pitch, 1.0f);
  const float ca = std::cos(settings_.tone_angle), sa = std::sin(settings_.tone_angle);
  stamp_along(p, [&](const StampInfo& s) {
    Stamp st;
    if(!make_stamp(s, st)) return;
    for(int j = 0; j < st.h; j++) {
      for(int i = 0; i < st.w; i++) {
        const float density = std::min(1.0f, st.coverage[(size_t)j * st.w + i] * s.flow);
        if(density <= 0.0f) continue;
        const int x = st.x0 + i, y = st.y0 + j;
        // 網点の格子(角度tone_angle)の中心からの距離が、濃さに応じた半径以内なら塗る(面積が濃さに比例、1で全面)
        const float xr = ((x + 0.5f) * ca + (y + 0.5f) * sa) / pitch, yr = (-(x + 0.5f) * sa + (y + 0.5f) * ca) / pitch;
        const float u = xr - std::floor(xr) - 0.5f, v = yr - std::floor(yr) - 0.5f;
        if(std::hypot(u, v) <= std::sqrt(density) * 0.7071f) blend_over_straight(target_->at(x, y), settings_.color, settings_.color[3] / 255.0f);
      }
    }
  });
}

void ParticleStroke::add(const PenSample& p) {
  stamp_along(p, [&](const StampInfo& s) {
    for(int k = 0; k < settings_.particle_count; k++) {
      StampInfo q   = s;
      const float r = s.radius * std::sqrt(rnd01()), a = rnd01() * 6.2831853f;
      q.x += r * std::cos(a);
      q.y += r * std::sin(a);
      q.radius = std::max(0.5f, s.radius * settings_.particle_size * (0.5f + 0.5f * rnd01()));
      paint(q);
    }
  });
}

void WatercolorStroke::add(const PenSample& p) {
  stamp_along(p, [&](const StampInfo& s) {
    Stamp st;
    if(!make_stamp(s, st)) return;
    for(int j = 0; j < st.h; j++) {
      for(int i = 0; i < st.w; i++) {
        const float cov = st.coverage[(size_t)j * st.w + i] * s.flow;
        if(cov <= 0.0f) continue;
        unsigned char& a = wet_.at(st.x0 + i, st.y0 + j)[3];
        a                = std::max(a, (unsigned char)std::clamp(cov * 255.0f + 0.5f, 0.0f, 255.0f)); // 重ねても濃くならない(水で薄まる)
      }
    }
  });
}

void WatercolorStroke::end() {
  const float mix = std::clamp(settings_.water_mix, 0.0f, 1.0f), edge = std::clamp(settings_.edge_darkening, 0.0f, 1.0f);
  const float ca = settings_.color[3] / 255.0f;
  for(const auto& [k, tile] : wet_.tiles()) {
    const int ox = TiledImage::key_tx(k) * TiledImage::kTile, oy = TiledImage::key_ty(k) * TiledImage::kTile;
    for(int j = 0; j < TiledImage::kTile; j++) {
      for(int i = 0; i < TiledImage::kTile; i++) {
        const float d = tile(i, j)[3] / 255.0f;
        if(d <= 0.0f) continue;
        const int x = ox + i, y = oy + j;
        if(!target_->in_range(x, y)) continue;
        const float dd    = std::min(1.0f, d * (1.0f + edge * (1.0f - d))); // 淡い所(=縁)ほど相対的に濃くする
        const Vec4b under = target_->get(x, y);
        Vec4b c           = settings_.color;
        const float m     = mix * under[3] / 255.0f; // 下地の色が混ざる量
        for(int ch = 0; ch < 3; ch++) c[ch] = (unsigned char)(settings_.color[ch] + (under[ch] - settings_.color[ch]) * m + 0.5f);
        blend_over_straight(target_->at(x, y), c, dd * ca);
      }
    }
  }
}

} // namespace mu
