#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/shape.hpp>

using namespace mu;

namespace {
Ref<Composition> make_test_comp(int w, int h) { return cutil::make_ref<Composition>("test", w, h, 30); }

Image make_target(int w, int h) {
  Image target;
  target.resize(w, h);
  target.has_alpha = false;
  target.fill_rgba(Vec4b(0, 0, 0, 255));
  return target;
}
} // namespace

TEST_CASE("ShapeEntt::render 四角形を指定位置・指定色で描画する") {
  auto comp   = make_test_comp(100, 80);
  auto target = make_target(100, 80);
  auto shp    = ShapeEntt::Create("rect", ShapeType_Rect);
  shp->pos_   = Vec3(10, 10, 0);
  shp->size_  = Vec2(30, 20);
  shp->color_ = Vec4b(255, 0, 0, 255);
  CHECK(shp->render(comp.get(), &target, 0));

  CHECK(target.rgba(20, 20) == Vec4b(255, 0, 0, 255)); // 矩形の内側
  CHECK(target.rgba(5, 5) == Vec4b(0, 0, 0, 255));     // 矩形の外側は背景のまま
}

TEST_CASE("ShapeEntt::render 枠線を指定色・太さで描画する") {
  auto comp          = make_test_comp(100, 80);
  auto target        = make_target(100, 80);
  auto shp           = ShapeEntt::Create("rect", ShapeType_Rect);
  shp->pos_          = Vec3(10, 10, 0);
  shp->size_         = Vec2(30, 20);
  shp->color_        = Vec4b(255, 0, 0, 255);
  shp->border_color_ = Vec4b(0, 255, 0, 255);
  shp->border_width_ = 3;
  CHECK(shp->render(comp.get(), &target, 0));

  CHECK(target.rgba(25, 20) == Vec4b(255, 0, 0, 255)); // 内側は塗り色のまま
  CHECK(target.rgba(10, 20) == Vec4b(0, 255, 0, 255)); // 左端の枠線
  CHECK(target.rgba(1, 1) == Vec4b(0, 0, 0, 255));     // 枠線の外は背景のまま
}

TEST_CASE("ShapeEntt::render 円は矩形の隅を塗らない") {
  auto comp   = make_test_comp(60, 60);
  auto target = make_target(60, 60);
  auto shp    = ShapeEntt::Create("circ", ShapeType_Circle);
  shp->pos_   = Vec3(0, 0, 0);
  shp->size_  = Vec2(50, 50);
  shp->color_ = Vec4b(0, 255, 0, 255);
  CHECK(shp->render(comp.get(), &target, 0));

  CHECK(target.rgba(25, 25) == Vec4b(0, 255, 0, 255)); // 円の中心
  CHECK(target.rgba(1, 1) == Vec4b(0, 0, 0, 255));     // バウンディングボックスの隅は円の外
}

TEST_CASE("ShapeEntt::render カスタムパスは点群のbboxに合わせて配置される") {
  auto comp        = make_test_comp(100, 100);
  auto target      = make_target(100, 100);
  auto shp         = ShapeEntt::Create("custom", ShapeType_Custom);
  shp->pos_        = Vec3(0, 0, 0);
  shp->custom_path = "10,10;40,10;40,40;10,40"; // 30x30の正方形
  shp->color_      = Vec4b(0, 0, 255, 255);
  CHECK(shp->render(comp.get(), &target, 0));

  CHECK(target.rgba(25, 25) == Vec4b(0, 0, 255, 255)); // パス内部
  CHECK(target.rgba(1, 1) == Vec4b(0, 0, 0, 255));     // パス外
}
