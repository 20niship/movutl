#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/render2d/renderer.hpp>

using namespace mu;

namespace {
Ref<Composition> make_test_comp(int w, int h) { return cutil::make_ref<Composition>("test", w, h, 30); }
} // namespace

TEST_CASE("clipping_up: 下に何も無い領域では上のオブジェクトが描画されない(#35)") {
  auto comp      = make_test_comp(60, 60);
  comp->bg_color = 0; // 透明背景にしないと下の合成済みアルファが常に255(不透明黒)になりclippingの効果を観測できない

  auto bottom        = ShapeEntt::Create("bottom", ShapeType_Rect);
  bottom->pos_       = Vec3(0, 0, 0); // 左半分(x:0-30)だけに配置
  bottom->size_      = Vec2(30, 60);
  bottom->color_     = Vec4b(255, 0, 0, 255);
  bottom->trk.fstart = 0;
  bottom->trk.fend   = 10;

  auto top             = ShapeEntt::Create("top", ShapeType_Rect);
  top->pos_            = Vec3(0, 0, 0); // 全面(x:0-60)
  top->size_           = Vec2(60, 60);
  top->color_          = Vec4b(0, 0, 255, 255);
  top->trk.fstart      = 0;
  top->trk.fend        = 10;
  top->trk.clipping_up = true;

  comp->insert_entity(bottom, 0);
  comp->insert_entity(top, 1);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp.get(), 0, out));

  CHECK(out->rgba(15, 30) == Vec4b(0, 0, 255, 255)); // 左半分(下にbottomがある): topが見える
  CHECK(out->rgba(45, 30) == Vec4b(0, 0, 0, 0));     // 右半分(下に何も無い): topはクリップされ透明のまま
}

TEST_CASE("clipping_up=falseなら通常通り全面に描画される") {
  auto comp = make_test_comp(60, 60);

  auto bottom        = ShapeEntt::Create("bottom", ShapeType_Rect);
  bottom->pos_       = Vec3(-15, 0, 0);
  bottom->size_      = Vec2(30, 60);
  bottom->color_     = Vec4b(255, 0, 0, 255);
  bottom->trk.fstart = 0;
  bottom->trk.fend   = 10;

  auto top        = ShapeEntt::Create("top2", ShapeType_Rect);
  top->pos_       = Vec3(0, 0, 0);
  top->size_      = Vec2(60, 60);
  top->color_     = Vec4b(0, 0, 255, 255);
  top->trk.fstart = 0;
  top->trk.fend   = 10;

  comp->insert_entity(bottom, 0);
  comp->insert_entity(top, 1);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp.get(), 0, out));

  CHECK(out->rgba(15, 30) == Vec4b(0, 0, 255, 255));
  CHECK(out->rgba(45, 30) == Vec4b(0, 0, 255, 255)); // クリップ無しなので右半分にも描画される
}

TEST_CASE("group_guid: getSaveProps/fromSavePropsで保存/復元される(#35)") {
  auto shp            = ShapeEntt::Create("g", ShapeType_Rect);
  shp->trk.group_guid = 42;

  auto saved    = shp->getSaveProps();
  auto restored = Entity::fromSaveProps(saved);
  REQUIRE(restored != nullptr);
  CHECK(restored->trk.group_guid == 42);
}
