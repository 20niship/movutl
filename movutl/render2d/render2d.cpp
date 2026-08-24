#include <movutl/asset/image.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/time.hpp>
#include <movutl/render2d/render2d.hpp>

namespace mu {

bool render_comp(Composition* comp) {
  MOVUTL_ZONE_SCOPED_N("render_comp");
  MU_ASSERT(comp != nullptr);
  if(!comp->frame_final) {
    comp->frame_final = cutil::make_ref<Image>(comp->size[0], comp->size[1]);
  } else if(comp->frame_final->width != comp->size[0] || comp->frame_final->height != comp->size[1]) {
    comp->frame_final->resize(Vec2d(comp->size[0], comp->size[1]));
  }
  uint32_t bg = (uint32_t)comp->bg_color;
  comp->frame_final->fill_rgba(Vec4b{(unsigned char)(bg & 0xFF), (unsigned char)((bg >> 8) & 0xFF), (unsigned char)((bg >> 16) & 0xFF), (unsigned char)((bg >> 24) & 0xFF)});

  int frame = comp->frame;
  for(auto& layer : comp->layers) {
    if(!layer.active) continue;
    auto e = layer.find_entt(frame);
    if(!e) continue;
    e->render(comp);
  }
  comp->frame_final->dirty();
  return true;
}

} // namespace mu
