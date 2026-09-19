#include <doctest/doctest.h>
#include <movutl/gui/transform_gizmo.hpp>

using namespace mu;

namespace {
const GizmoPt kComp{200, 100};
const GizmoPt kNoOff{0, 0};
} // namespace

TEST_CASE("gizmo_quad: 基点0・回転0では画像がコンポ中心にposぶんずれて置かれる") {
  GizmoXform x;
  x.pos  = {10, -5};
  auto q = gizmo_quad(x, {40, 20}, kNoOff, kComp);
  CHECK(q.p[0].x == doctest::Approx(90)); // 100+10-20
  CHECK(q.p[0].y == doctest::Approx(35)); // 50-5-10
  CHECK(q.p[2].x == doctest::Approx(130));
  CHECK(q.p[2].y == doctest::Approx(55));
}

TEST_CASE("gizmo_quad: 基点は回転の中心になり、基点マーカー位置はposで決まる") {
  GizmoXform x;
  x.anchor = {20, 10}; // 右下の角
  x.rot    = 90;
  auto ap  = gizmo_anchor_point(x, kComp);
  CHECK(ap.x == doctest::Approx(100));
  CHECK(ap.y == doctest::Approx(50));
  auto q = gizmo_quad(x, {40, 20}, kNoOff, kComp);
  CHECK(q.p[2].x == doctest::Approx(100)); // 基点(=右下の角)は回転しても動かない
  CHECK(q.p[2].y == doctest::Approx(50));
  CHECK(q.p[0].x == doctest::Approx(120)); // 左上(-20,-10)-基点(20,10)=(-40,-20) を時計回り90度 => (20,-40)
  CHECK(q.p[0].y == doctest::Approx(10));
}

TEST_CASE("gizmo_quad: origin_offsetは基点の既定位置をずらす(anchorに加算されるのと同じ)") {
  GizmoXform a, b;
  a.anchor = {5, 3};
  a.rot    = 30;
  b.rot    = 30;
  auto qa  = gizmo_quad(a, {40, 20}, {0, 0}, kComp);
  auto qb  = gizmo_quad(b, {40, 20}, {5, 3}, kComp);
  for(int i = 0; i < 4; i++) {
    CHECK(qa.p[i].x == doctest::Approx(qb.p[i].x));
    CHECK(qa.p[i].y == doctest::Approx(qb.p[i].y));
  }
}

TEST_CASE("gizmo_comp_to_local は gizmo_local_to_comp の逆変換") {
  GizmoXform x;
  x.pos    = {12, -7};
  x.anchor = {4, 9};
  x.scale  = {150, 60};
  x.rot    = 37;
  GizmoPt off{3, -2}, l{11, -6};
  auto c = gizmo_local_to_comp(x, off, kComp, l);
  auto b = gizmo_comp_to_local(x, off, kComp, c);
  CHECK(b.x == doctest::Approx(l.x));
  CHECK(b.y == doctest::Approx(l.y));
}

TEST_CASE("gizmo_point_in_quad: 回転した四角形の内外判定") {
  GizmoXform x;
  x.rot  = 45;
  auto q = gizmo_quad(x, {40, 40}, kNoOff, kComp);
  CHECK(gizmo_point_in_quad(q, {100, 50}));
  CHECK(gizmo_point_in_quad(q, {100 + 25, 50}));            // 45度回転で対角線が水平になる(半径約28)
  CHECK_FALSE(gizmo_point_in_quad(q, {100 + 19, 50 + 19})); // 元の角の位置(回転後は外)
}

TEST_CASE("gizmo_hit_test: 基点>回転ハンドル>角>本体の優先順位") {
  GizmoXform x;
  auto q  = gizmo_quad(x, {40, 20}, kNoOff, kComp);
  auto ap = gizmo_anchor_point(x, kComp);
  CHECK(gizmo_hit_test(q, ap, {100, 50}, 4, 24).part == GizmoPart::Anchor); // 中心=基点は本体より優先
  auto h = gizmo_hit_test(q, ap, {q.p[2].x, q.p[2].y}, 4, 24);
  CHECK(h.part == GizmoPart::Scale);
  CHECK(h.corner == 2);
  CHECK(gizmo_hit_test(q, ap, {100, 40 - 24}, 4, 24).part == GizmoPart::Rotate); // 上辺(y=40)の24px上
  CHECK(gizmo_hit_test(q, ap, {110, 55}, 4, 24).part == GizmoPart::Body);
  CHECK(gizmo_hit_test(q, ap, {0, 0}, 4, 24).part == GizmoPart::None);
}

TEST_CASE("gizmo_rotate_handle: 回転しても画像の上方向に出る") {
  GizmoXform x;
  x.rot  = 90; // 時計回り90度: 画像の上は画面の右
  auto q = gizmo_quad(x, {40, 20}, kNoOff, kComp);
  auto h = gizmo_rotate_handle(q, 24);
  CHECK(h.x == doctest::Approx(100 + 10 + 24));
  CHECK(h.y == doctest::Approx(50));
}

TEST_CASE("gizmo_drag_move / gizmo_drag_rotate") {
  GizmoXform x;
  x.pos  = {1, 2};
  auto m = gizmo_drag_move(x, {10, 10}, {25, 4});
  CHECK(m.pos.x == doctest::Approx(16));
  CHECK(m.pos.y == doctest::Approx(-4));
  GizmoXform o;
  auto r = gizmo_drag_rotate(o, kComp, {200, 50}, {100, 150}); // 基点(100,50)まわりで右方向から下方向へ => 時計回り+90度
  CHECK(r.rot == doctest::Approx(90));
}

TEST_CASE("gizmo_drag_scale: 角をドラッグすると基点からの距離比で拡大率が決まる") {
  GizmoXform x;
  GizmoPt corner{20, 10}; // 右下
  // 基点(100,50)から角(120,60)を、(140,70)へ動かす => 2倍
  auto s = gizmo_drag_scale(x, kNoOff, kComp, corner, {140, 70}, false);
  CHECK(s.scale.x == doctest::Approx(200));
  CHECK(s.scale.y == doctest::Approx(200));
  auto s2 = gizmo_drag_scale(x, kNoOff, kComp, corner, {140, 60}, false); // 縦は変えず横だけ2倍
  CHECK(s2.scale.x == doctest::Approx(200));
  CHECK(s2.scale.y == doctest::Approx(100));
  auto s3 = gizmo_drag_scale(x, kNoOff, kComp, corner, {140, 60}, true); // 縦横比維持
  CHECK(s3.scale.x == doctest::Approx(200));
  CHECK(s3.scale.y == doctest::Approx(200));
}

TEST_CASE("gizmo_set_anchor: keep_visualなら見た目(画像の位置)が変わらない") {
  GizmoXform x;
  x.pos       = {5, 5};
  x.scale     = {200, 50};
  x.rot       = 30;
  auto before = gizmo_quad(x, {40, 20}, kNoOff, kComp);
  auto y      = gizmo_set_anchor(x, {12, -4}, true);
  auto after  = gizmo_quad(y, {40, 20}, kNoOff, kComp);
  for(int i = 0; i < 4; i++) {
    CHECK(after.p[i].x == doctest::Approx(before.p[i].x));
    CHECK(after.p[i].y == doctest::Approx(before.p[i].y));
  }
  auto z = gizmo_set_anchor(x, {12, -4}, false); // 補正しない場合はposがそのまま
  CHECK(z.pos.x == doctest::Approx(x.pos.x));
  CHECK(z.pos.y == doctest::Approx(x.pos.y));
}

TEST_CASE("gizmo_anchor_preset / gizmo_to_center_origin") {
  auto p = gizmo_anchor_preset(-1, 1, {40, 20}, {0, 0}); // 左下
  CHECK(p.x == doctest::Approx(-20));
  CHECK(p.y == doctest::Approx(10));
  auto p2 = gizmo_anchor_preset(1, 0, {40, 20}, {5, 2}); // origin_offset分だけ引く
  CHECK(p2.x == doctest::Approx(15));
  CHECK(p2.y == doctest::Approx(-2));
  auto c = gizmo_to_center_origin({100, 50}, kComp);
  CHECK(c.x == doctest::Approx(0));
  CHECK(c.y == doctest::Approx(0));
}
