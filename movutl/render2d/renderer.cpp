#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

bool CPURenderer::render_frame(Composition* comp, int frame, Ref<Image>& out) {
  MOVUTL_ZONE_SCOPED_N("CPURenderer::render_frame");
  MU_ASSERT(comp != nullptr);

  if(!out) {
    out = cutil::make_ref<Image>(comp->size[0], comp->size[1]);
  } else if(out->width != comp->size[0] || out->height != comp->size[1]) {
    MOVUTL_ZONE_SCOPED_N("CPURenderer::resize");
    out->resize(Vec2d(comp->size[0], comp->size[1]));
  }

  {
    uint32_t bg = (uint32_t)comp->bg_color;
    MOVUTL_ZONE_SCOPED_N("CPURenderer::resize");
    out->fill_rgba(Vec4b{(unsigned char)(bg & 0xFF), (unsigned char)((bg >> 8) & 0xFF), (unsigned char)((bg >> 16) & 0xFF), (unsigned char)((bg >> 24) & 0xFF)});
  }

  // comp->mtxはget_all_entities()内で短時間lockするのみ。Entity個々のレンダリング中はe->mtxだけをlockする
  for(auto& e : comp->get_all_entities()) {
    std::lock_guard<std::mutex> lock(e->mtx);
    if(!e->visible(frame)) continue;

    if(e->trk.clipping_up) {
      // ponytail: 単一共有バッファ逐次合成のため未描画の上レイヤーは参照不可。既に合成済みの下側アルファをマスクに使う近似実装(真の上レイヤークリッピングには2パスレンダリングが必要)
      Image scratch(out->width, out->height);
      scratch.fill_rgba(Vec4b(0, 0, 0, 0));
      e->render(comp, &scratch, frame);
      for(size_t i = 0; i < out->size(); i++) {
        Vec4b s    = scratch[i];
        Vec4b& d   = (*out)[i];
        float mask = d[3] / 255.0f; // 既存(下)のアルファをマスクにする
        float a    = (s[3] / 255.0f) * mask;
        if(a <= 0.0f) continue;
        for(int c = 0; c < 3; c++) d[c] = (unsigned char)(s[c] * a + d[c] * (1.0f - a));
        d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
      }
    } else {
      e->render(comp, out.get(), frame);
    }
  }
  out->dirty();
  return true;
}

} // namespace mu
