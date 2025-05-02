#include <movutl/asset/image.hpp>
#include <movutl/core/imagebase.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/time.hpp>
#include <movutl/render2d/render2d.hpp>

namespace mu {

bool render_comp(Composition* comp) {
  MU_ASSERT(comp != nullptr);
  if(!comp->frame_final) {
    comp->frame_final = std::make_shared<ImageRGBA>(comp->size[0], comp->size[1]);
  } else if(comp->frame_final->width != comp->size[0] || comp->frame_final->height != comp->size[1]) {
    comp->frame_final->resize(Vec2d(comp->size[0], comp->size[1]));
  }
  comp->frame_final->fill(0);

  int frame = comp->frame;
  for(auto& layer : comp->layers) {
    if(!layer.active) continue;
    auto e = layer.find_entt(frame);
    if(!e) continue;
    e->render(comp);
  }
  return true;
}

} // namespace mu
