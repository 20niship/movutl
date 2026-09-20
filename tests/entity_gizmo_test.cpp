#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/gui/entity_gizmo.hpp>

using namespace mu;

TEST_CASE("entity_gizmo_of: Imageの枠はコンポ中心+posに画像サイズで置かれる") {
  Project::New();
  auto* cmp = Composition::GetActiveComp();
  REQUIRE(cmp != nullptr);
  const GizmoPt cs{(double)cmp->size[0], (double)cmp->size[1]};
  auto img = Image::Create("g", 40, 20);
  REQUIRE(img != nullptr);
  img->pos_[0] = 10;
  EntityGizmo g;
  REQUIRE(entity_gizmo_of(*img, cs, g));
  CHECK(g.quad.p[0].x == doctest::Approx(cs.x / 2 + 10 - 20));
  CHECK(g.quad.p[0].y == doctest::Approx(cs.y / 2 - 10));
  CHECK(g.anchor_pt.x == doctest::Approx(cs.x / 2 + 10));
}

TEST_CASE("hit_test_entity: 回転・拡大後の実描画枠で判定し、後ろのEntityが前面") {
  Project::New();
  auto* cmp = Composition::GetActiveComp();
  REQUIRE(cmp != nullptr);
  const double cx = cmp->size[0] / 2.0, cy = cmp->size[1] / 2.0;
  auto back     = Image::Create("back", 40, 40);
  auto front    = Image::Create("front", 40, 40);
  back->fstart_ = front->fstart_ = 0;
  back->fend_ = front->fend_ = 100;
  front->scale_              = 50.f; // 20x20に縮小
  cmp->insert_entity(back, 0);
  cmp->insert_entity(front, 1);
  cmp->frame = 10;
  CHECK(hit_test_entity(*cmp, {cx, cy}).get() == front.get());     // 重なる場所は前面
  CHECK(hit_test_entity(*cmp, {cx + 15, cy}).get() == back.get()); // frontの外・backの内
  CHECK(hit_test_entity(*cmp, {cx + 60, cy}).get() == nullptr);    // どちらの外
  back->pos_ = Vec3(200, 0, 0);                                    // 移動すれば元の位置は当たらない
  CHECK(hit_test_entity(*cmp, {cx + 15, cy}).get() == nullptr);
  CHECK(hit_test_entity(*cmp, {cx + 200, cy}).get() == back.get());
}

TEST_CASE("entity_apply_xform: 変換をEntityへ書き戻す") {
  auto shp = ShapeEntt::Create("s", ShapeType_Rect);
  GizmoXform x;
  x.pos    = {3, 4};
  x.anchor = {5, 6};
  x.scale  = 70;
  x.rot    = 15;
  entity_apply_xform(*shp, x);
  CHECK(shp->pos_[0] == doctest::Approx(3));
  CHECK(shp->anchor_[1] == doctest::Approx(6));
  CHECK(shp->scale_ == doctest::Approx(70));
  CHECK(shp->rotation_ == doctest::Approx(15));
}
