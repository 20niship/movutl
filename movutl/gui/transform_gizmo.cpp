#include <algorithm>
#include <movutl/gui/transform_gizmo.hpp>

namespace mu {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

GizmoPt rot(const GizmoPt& v, double deg) {
  const double r = deg * kDeg2Rad, c = std::cos(r), s = std::sin(r);
  return {v.x * c - v.y * s, v.x * s + v.y * c};
}
// 局所(画像中心原点・拡大前)の基点との差分を拡大→回転
GizmoPt scale_rot(const GizmoXform& x, const GizmoPt& d) { return rot({d.x * x.scale.x / 100.0, d.y * x.scale.y / 100.0}, x.rot); }
GizmoPt unscale_rot(const GizmoXform& x, const GizmoPt& d) {
  const GizmoPt v = rot(d, -x.rot);
  return {x.scale.x != 0 ? v.x * 100.0 / x.scale.x : 0, x.scale.y != 0 ? v.y * 100.0 / x.scale.y : 0};
}
double dist2(const GizmoPt& a, const GizmoPt& b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); }
} // namespace

GizmoPt gizmo_anchor_point(const GizmoXform& x, const GizmoPt& comp_size) { return {comp_size.x / 2 + x.pos.x, comp_size.y / 2 + x.pos.y}; }

GizmoPt gizmo_local_to_comp(const GizmoXform& x, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& local) {
  const GizmoPt a = x.anchor + origin_offset;
  return gizmo_anchor_point(x, comp_size) + scale_rot(x, local - a);
}

GizmoPt gizmo_comp_to_local(const GizmoXform& x, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& comp) {
  const GizmoPt a = x.anchor + origin_offset;
  return a + unscale_rot(x, comp - gizmo_anchor_point(x, comp_size));
}

GizmoQuad gizmo_quad(const GizmoXform& x, const GizmoPt& src_size, const GizmoPt& origin_offset, const GizmoPt& comp_size) {
  const double hw = src_size.x / 2, hh = src_size.y / 2;
  const GizmoPt corners[4] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
  GizmoQuad q;
  for(int i = 0; i < 4; i++) q.p[i] = gizmo_local_to_comp(x, origin_offset, comp_size, corners[i]);
  return q;
}

bool gizmo_point_in_quad(const GizmoQuad& q, const GizmoPt& pt) {
  bool pos = false, neg = false;
  for(int i = 0; i < 4; i++) {
    const GizmoPt a = q.p[i], b = q.p[(i + 1) % 4];
    const double cr = (b.x - a.x) * (pt.y - a.y) - (b.y - a.y) * (pt.x - a.x);
    if(cr > 0) pos = true;
    if(cr < 0) neg = true;
  }
  return !(pos && neg); // 全辺に対して同じ側なら内側(辺上も内側扱い)
}

GizmoPt gizmo_rotate_handle(const GizmoQuad& q, double dist) {
  const GizmoPt mid = {(q.p[0].x + q.p[1].x) / 2, (q.p[0].y + q.p[1].y) / 2};
  const GizmoPt bot = {(q.p[2].x + q.p[3].x) / 2, (q.p[2].y + q.p[3].y) / 2};
  const GizmoPt up  = mid - bot; // 下辺中点から上辺中点へ向かう向き(画像の「上」方向)
  const double len  = std::sqrt(up.x * up.x + up.y * up.y);
  if(len < 1e-9) return {mid.x, mid.y - dist};
  return {mid.x + up.x / len * dist, mid.y + up.y / len * dist};
}

GizmoHit gizmo_hit_test(const GizmoQuad& q, const GizmoPt& anchor_pt, const GizmoPt& pt, double hit_r, double rot_dist) {
  const double r2 = hit_r * hit_r;
  if(dist2(anchor_pt, pt) <= r2) return {GizmoPart::Anchor, -1};
  if(dist2(gizmo_rotate_handle(q, rot_dist), pt) <= r2) return {GizmoPart::Rotate, -1};
  for(int i = 0; i < 4; i++)
    if(dist2(q.p[i], pt) <= r2) return {GizmoPart::Scale, i};
  if(gizmo_point_in_quad(q, pt)) return {GizmoPart::Body, -1};
  return {};
}

GizmoXform gizmo_drag_move(const GizmoXform& s0, const GizmoPt& m0, const GizmoPt& m) {
  GizmoXform r = s0;
  r.pos        = s0.pos + (m - m0);
  return r;
}

GizmoXform gizmo_drag_scale(const GizmoXform& s0, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& corner_local, const GizmoPt& m, bool uniform) {
  GizmoXform r    = s0;
  const GizmoPt a = s0.anchor + origin_offset;
  const GizmoPt c = corner_local - a;                                    // 基点から角への拡大前ベクトル
  const GizmoPt v = rot(m - gizmo_anchor_point(s0, comp_size), -s0.rot); // 基点からマウスへの、回転を戻したベクトル
  double sx       = std::abs(c.x) > 1e-6 ? v.x / c.x * 100.0 : s0.scale.x;
  double sy       = std::abs(c.y) > 1e-6 ? v.y / c.y * 100.0 : s0.scale.y;
  if(uniform) {
    // 開始時の縦横比を保つ: 開始スケールに対する倍率の大きい方を採用
    const double kx = s0.scale.x != 0 ? sx / s0.scale.x : 1, ky = s0.scale.y != 0 ? sy / s0.scale.y : 1;
    const double k  = std::abs(kx) > std::abs(ky) ? kx : ky;
    sx              = s0.scale.x * k;
    sy              = s0.scale.y * k;
  }
  r.scale = {sx, sy};
  return r;
}

GizmoXform gizmo_drag_rotate(const GizmoXform& s0, const GizmoPt& comp_size, const GizmoPt& m0, const GizmoPt& m) {
  const GizmoPt ap = gizmo_anchor_point(s0, comp_size);
  const double a0  = std::atan2(m0.y - ap.y, m0.x - ap.x);
  const double a1  = std::atan2(m.y - ap.y, m.x - ap.x);
  GizmoXform r     = s0;
  r.rot            = s0.rot + (a1 - a0) / kDeg2Rad;
  return r;
}

GizmoXform gizmo_set_anchor(const GizmoXform& s0, const GizmoPt& new_anchor, bool keep_visual) {
  GizmoXform r = s0;
  r.anchor     = new_anchor;
  // 画像中心の描画位置は pos - RS*anchor。anchorを動かして見た目を保つには pos を RS*(new-old) だけ動かす
  if(keep_visual) r.pos = s0.pos + scale_rot(s0, new_anchor - s0.anchor);
  return r;
}

GizmoPt gizmo_anchor_preset(int fx, int fy, const GizmoPt& src_size, const GizmoPt& origin_offset) { return {fx * src_size.x / 2 - origin_offset.x, fy * src_size.y / 2 - origin_offset.y}; }

GizmoPt gizmo_to_center_origin(const GizmoPt& p, const GizmoPt& comp_size) { return {p.x - comp_size.x / 2, p.y - comp_size.y / 2}; }

} // namespace mu
