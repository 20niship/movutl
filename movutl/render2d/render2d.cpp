#include <movutl/core/profiler.hpp>
#include <movutl/render2d/render2d.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

bool render_comp(Composition* comp) {
  MOVUTL_ZONE_SCOPED_N("render_comp");
  MU_ASSERT(comp != nullptr);
  static thread_local CPURenderer renderer;
  Ref<Image> out = comp->frame_final;
  bool ok        = renderer.render_frame(comp, comp->frame, out);
  comp->frame_final = out;
  return ok;
}

} // namespace mu
