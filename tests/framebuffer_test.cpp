#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/render2d/renderer.hpp>

using namespace mu;

namespace {
Ref<Composition> make_test_comp(int w, int h) { return cutil::make_ref<Composition>("test", w, h, 30); }

Ref<Image> make_target(int w, int h) {
  auto target = cutil::make_ref<Image>();
  target->resize(w, h);
  target->has_alpha = false;
  target->fill_rgba(Vec4b(0, 0, 0, 255));
  return target;
}
} // namespace

TEST_CASE("FramebufferEntt::render targetの現在の内容をそのままキャプチャする") {
  auto comp   = make_test_comp(20, 20);
  auto target = make_target(20, 20);
  target->set_rgba(5, 5, Vec4b(10, 20, 30, 255));
  auto fb = FramebufferEntt::Create("fb");
  CHECK(fb->render(comp.get(), target.get(), 0));

  REQUIRE(fb->captured_image() != nullptr);
  CHECK(fb->captured_image()->width == 20);
  CHECK(fb->captured_image()->height == 20);
  CHECK(fb->captured_image()->rgba(5, 5) == Vec4b(10, 20, 30, 255));
}

TEST_CASE("FramebufferEntt::render clear_original_=falseならtargetは変更されない") {
  auto comp           = make_test_comp(10, 10);
  auto target         = make_target(10, 10);
  auto before         = target->rgba(3, 3);
  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = false;
  CHECK(fb->render(comp.get(), target.get(), 0));
  CHECK(target->rgba(3, 3) == before);
}

TEST_CASE("FramebufferEntt::render clear_original_=trueならtargetが透明にクリアされる") {
  auto comp           = make_test_comp(10, 10);
  auto target         = make_target(10, 10);
  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = true;
  CHECK(fb->render(comp.get(), target.get(), 0));
  for(int y = 0; y < 10; y++)
    for(int x = 0; x < 10; x++) CHECK(target->rgba(x, y) == Vec4b(0, 0, 0, 0));
}

TEST_CASE("FramebufferEntt: レイヤーをまたいでグループ化的に使える(下のレイヤーをキャプチャしてクリアし、上のレイヤーだけが最終出力に残る)") {
  auto comp = make_test_comp(60, 60);

  auto bottom        = ShapeEntt::Create("bottom", ShapeType_Rect);
  bottom->pos_       = Vec3(0, 0, 0);
  bottom->size_      = Vec2(60, 60);
  bottom->color_     = Vec4b(255, 0, 0, 255); // 赤
  bottom->trk.fstart = 0;
  bottom->trk.fend   = 10;

  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = true;
  fb->trk.fstart      = 0;
  fb->trk.fend        = 10;

  auto top        = ShapeEntt::Create("top", ShapeType_Rect);
  top->pos_       = Vec3(0, 0, 0);
  top->size_      = Vec2(20, 20);
  top->color_     = Vec4b(0, 255, 0, 255); // 緑
  top->trk.fstart = 0;
  top->trk.fend   = 10;

  comp->insert_entity(bottom, 0);
  comp->insert_entity(fb, 1);
  comp->insert_entity(top, 2);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp.get(), 0, out));

  CHECK(out->rgba(10, 10) == Vec4b(0, 255, 0, 255)); // 上の図形の内側: 緑
  CHECK(out->rgba(40, 40) == Vec4b(0, 0, 0, 0));     // clear_original_により下の赤は消え透明になっている

  REQUIRE(fb->captured_image() != nullptr);
  CHECK(fb->captured_image()->rgba(40, 40) == Vec4b(255, 0, 0, 255)); // キャプチャ画像には下の赤が残っている
}
