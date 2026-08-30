#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

bool CPURenderer::render_frame(Composition* comp, int frame, Ref<Image>& out) {
  MOVUTL_ZONE_SCOPED_N("CPURenderer::render_frame");
  MU_ASSERT(comp != nullptr);
  std::lock_guard<std::mutex> lock(comp->mtx);

  if(!out) {
    out = cutil::make_ref<Image>(comp->size[0], comp->size[1]);
  } else if(out->width != comp->size[0] || out->height != comp->size[1]) {
    out->resize(Vec2d(comp->size[0], comp->size[1]));
  }

  int saved_frame        = comp->frame;
  Ref<Image> saved_final = comp->frame_final;
  comp->frame            = frame;
  comp->frame_final      = out;

  uint32_t bg = (uint32_t)comp->bg_color;
  comp->frame_final->fill_rgba(Vec4b{(unsigned char)(bg & 0xFF), (unsigned char)((bg >> 8) & 0xFF), (unsigned char)((bg >> 16) & 0xFF), (unsigned char)((bg >> 24) & 0xFF)});

  for(auto& layer : comp->layers) {
    if(!layer.active) continue;
    auto e = layer.find_entt(frame);
    if(!e) continue;
    e->render(comp);
  }
  comp->frame_final->dirty();
  out = comp->frame_final;

  comp->frame       = saved_frame;
  comp->frame_final = saved_final;
  return true;
}

} // namespace mu
