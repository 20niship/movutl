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
    e->render(comp, out.get(), frame);
  }
  out->dirty();
  return true;
}

} // namespace mu
