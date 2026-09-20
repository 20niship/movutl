#include <movutl/gui/entity_gizmo.hpp>

namespace mu {

bool entity_gizmo_of(const Entity& e, const GizmoPt& comp_size, EntityGizmo& out) {
  if(!e.has_transform()) return false;
  out.comp_size    = comp_size;
  out.xform.pos    = {e.pos_[0], e.pos_[1]};
  out.xform.anchor = {e.anchor_[0], e.anchor_[1]};
  out.xform.scale  = e.scale_;
  out.xform.rot    = e.rotation_;
  Vec2 size, origin;
  if(e.source_size(size, origin)) {
    out.src_size      = {size[0], size[1]};
    out.origin_offset = {origin[0], origin[1]};
  } else {
    out.src_size      = comp_size;
    out.origin_offset = {0, 0};
  }
  out.quad      = gizmo_quad(out.xform, out.src_size, out.origin_offset, comp_size);
  out.anchor_pt = gizmo_anchor_point(out.xform, comp_size);
  return true;
}

void entity_apply_xform(Entity& e, const GizmoXform& x) {
  e.pos_[0]    = (float)x.pos.x;
  e.pos_[1]    = (float)x.pos.y;
  e.anchor_[0] = (float)x.anchor.x;
  e.anchor_[1] = (float)x.anchor.y;
  e.scale_     = (float)x.scale;
  e.rotation_  = (float)x.rot;
}

Ref<Entity> hit_test_entity(const Composition& cmp, const GizmoPt& comp_pt) {
  const GizmoPt comp_size{(double)cmp.size[0], (double)cmp.size[1]};
  Ref<Entity> hit;
  for(auto& e : cmp.get_all_entities()) {
    EntityGizmo g;
    if(!e->visible(cmp.frame) || !entity_gizmo_of(*e, comp_size, g)) continue;
    if(gizmo_point_in_quad(g.quad, comp_pt)) hit = e;
  }
  return hit;
}

} // namespace mu
