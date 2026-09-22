#include <movutl/asset/scene_change.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/composite_ops.hpp>

namespace mu {

void composite_scene_change(Composition* comp, Entity* a, Entity* e, const SceneChangeEntt* sc, Image* out, int frame) {
  MOVUTL_ZONE_SCOPED_N("composite_scene_change");
  Image sa(out->width, out->height), sb(out->width, out->height);
  sa.fill_rgba(Vec4b(0, 0, 0, 0));
  sb.fill_rgba(Vec4b(0, 0, 0, 0));
  if(a) {
    std::lock_guard<std::mutex> la(a->mtx);
    a->apply_animated_props(a->fend_);
    a->render(comp, &sa, a->fend_);
  }
  e->render(comp, &sb, frame);
  const float p = sc->progress(frame);
  const int W = out->width, H = out->height;
  for(size_t i = 0; i < out->size(); i++) {
    const float w   = SceneChangeWeight(sc->type_, p, ((int)(i % W) + 0.5f) / W, ((int)(i / W) + 0.5f) / H, sc->invert_, sc->blur_);
    const Vec4b &ca = sa[i], &cb = sb[i];
    const float aa = ca[3] / 255.0f * (1.0f - w), ab = cb[3] / 255.0f * w, at = aa + ab;
    if(at <= 0.0f) continue;
    Vec4b& d = (*out)[i];
    for(int c = 0; c < 3; c++) d[c] = (unsigned char)((ca[c] * aa + cb[c] * ab) + d[c] * (1.0f - at));
    d[3] = (unsigned char)(at * 255.0f + d[3] * (1.0f - at));
  }
}

void composite_clipping_up(Composition* comp, Entity* e, const Image& mask, Image* out, int frame) {
  MOVUTL_ZONE_SCOPED_N("composite_clipping_up");
  Image scratch(out->width, out->height);
  scratch.fill_rgba(Vec4b(0, 0, 0, 0));
  e->render(comp, &scratch, frame);
  for(size_t i = 0; i < out->size(); i++) {
    Vec4b s  = scratch[i];
    Vec4b& d = (*out)[i];
    float a  = (s[3] / 255.0f) * (mask[i][3] / 255.0f);
    if(a <= 0.0f) continue;
    for(int c = 0; c < 3; c++) d[c] = (unsigned char)(s[c] * a + d[c] * (1.0f - a));
    d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
  }
}

} // namespace mu
