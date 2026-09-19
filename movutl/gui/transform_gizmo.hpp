#pragma once
#include <cmath>

namespace mu {

// ビューア変換ギズモの幾何ロジック(ImGui非依存)。座標はコンポ左上原点・Y下向きpxで、Entity::composite()と同じ変換式を使う
struct GizmoPt {
  double x = 0, y = 0;
  GizmoPt operator+(const GizmoPt& o) const { return {x + o.x, y + o.y}; }
  GizmoPt operator-(const GizmoPt& o) const { return {x - o.x, y - o.y}; }
};

// 描画変換(Entityのpos_/anchor_/scale_/rotation_の2D成分)。scaleは%、rotは度(時計回りが正)
struct GizmoXform {
  GizmoPt pos;
  GizmoPt anchor;
  GizmoPt scale = {100, 100};
  double rot    = 0;
};

// 表示される画像の枠。p[]は 左上,右上,右下,左下(回転前の並び)
struct GizmoQuad {
  GizmoPt p[4];
};

// 画像中心を基準にした局所座標(px, 拡大前) -> コンポ左上原点座標。origin_offsetは基点の既定位置のずれ(Shape等)
GizmoPt gizmo_local_to_comp(const GizmoXform& x, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& local);
// コンポ座標 -> 画像中心の局所座標(gizmo_local_to_compの逆)
GizmoPt gizmo_comp_to_local(const GizmoXform& x, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& comp);

GizmoQuad gizmo_quad(const GizmoXform& x, const GizmoPt& src_size, const GizmoPt& origin_offset, const GizmoPt& comp_size);
// 基点マーカーの位置(=コンポ中心 + pos)
GizmoPt gizmo_anchor_point(const GizmoXform& x, const GizmoPt& comp_size);
bool gizmo_point_in_quad(const GizmoQuad& q, const GizmoPt& pt);

enum class GizmoPart { None, Body, Anchor, Scale, Rotate };
struct GizmoHit {
  GizmoPart part = GizmoPart::None;
  int corner     = -1; // Scaleのときの角(0-3、GizmoQuadのp[]の添字)
};

// 回転ハンドルの位置(上辺の中点から画像の上方向へ dist だけ離す)
GizmoPt gizmo_rotate_handle(const GizmoQuad& q, double dist);
// 優先順位: 基点 > 回転ハンドル > 角(拡大) > 本体。hit_rはハンドルの当たり半径、rot_distは回転ハンドルの距離(ともにコンポ座標px。画面px/zoomで換算して渡す)
GizmoHit gizmo_hit_test(const GizmoQuad& q, const GizmoPt& anchor_pt, const GizmoPt& pt, double hit_r, double rot_dist);

// ドラッグ操作(開始時のxform s0 と、開始時/現在のマウス位置(コンポ座標)から新しいxformを返す)
GizmoXform gizmo_drag_move(const GizmoXform& s0, const GizmoPt& m0, const GizmoPt& m);
// corner_local: 掴んだ角の画像中心局所座標(拡大前)。uniformで縦横比維持
GizmoXform gizmo_drag_scale(const GizmoXform& s0, const GizmoPt& origin_offset, const GizmoPt& comp_size, const GizmoPt& corner_local, const GizmoPt& m, bool uniform);
GizmoXform gizmo_drag_rotate(const GizmoXform& s0, const GizmoPt& comp_size, const GizmoPt& m0, const GizmoPt& m);
// 基点を局所座標new_anchor(=anchor_値。origin_offsetを含まない)へ移す。keep_visualなら見た目が動かないようposを補正する(AEのPan Behind)
GizmoXform gizmo_set_anchor(const GizmoXform& s0, const GizmoPt& new_anchor, bool keep_visual);
// 基点プリセット: fx,fy = -1(左/上) 0(中央) +1(右/下)。画像枠(src_size)上の点へ基点を置くときのanchor_値
GizmoPt gizmo_anchor_preset(int fx, int fy, const GizmoPt& src_size, const GizmoPt& origin_offset);

// コンポ左上原点 -> 中心原点(Y下向きのまま)。定規・カーソル座標表示用
GizmoPt gizmo_to_center_origin(const GizmoPt& p, const GizmoPt& comp_size);

} // namespace mu
