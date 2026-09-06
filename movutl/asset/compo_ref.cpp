#include <cmath>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

bool CompoRefEntt::render(Composition* cmp, Image* target, int frame) {
  MU_ASSERT(cmp);
  MU_ASSERT(target);
  Composition* dst_comp = nullptr;
  for(auto& c : Project::Get()->compos_) {
    if(c->guid == target_comp_guid) {
      dst_comp = c.get();
      break;
    }
  }
  if(!dst_comp || dst_comp == cmp) return false;

  dst_comp->frame.store(start_frame + (int32_t)std::lround((frame - trk.fstart) * speed));
  Ref<Image> src = dst_comp->render_current_frame_main_thread();
  if(!src) return false;

  int base_x = (int)pos[0];
  int base_y = (int)pos[1];
  return src->copyto(target, Vec2d(base_x, base_y), scale.avg(), rotation, alpha, trk.blend_);
}

} // namespace mu
