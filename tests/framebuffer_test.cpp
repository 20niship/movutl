#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/render2d/renderer.hpp>
#include <string>

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

TEST_CASE("FramebufferEntt::render デフォルトパラメータ(pos=0,scale=100,alpha=255)では貼り戻しにより見た目が変化しない") {
  auto comp   = make_test_comp(10, 10);
  auto target = make_target(10, 10);
  auto before = target->rgba(3, 3);
  auto fb     = FramebufferEntt::Create("fb");

  SUBCASE("clear_original_=false") { fb->clear_original_ = false; }
  SUBCASE("clear_original_=true") { fb->clear_original_ = true; } // クリアしても等倍で同じ位置に貼り戻すため結果的に見た目は変わらない

  CHECK(fb->render(comp.get(), target.get(), 0));
  CHECK(target->rgba(3, 3) == before);
}

TEST_CASE("FramebufferEntt::render alpha_=0で貼り戻しを無効化するとclear_original_の効果だけが残る") {
  auto comp           = make_test_comp(10, 10);
  auto target         = make_target(10, 10);
  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = true;
  fb->alpha_          = 0;
  CHECK(fb->render(comp.get(), target.get(), 0));
  for(int y = 0; y < 10; y++)
    for(int x = 0; x < 10; x++) CHECK(target->rgba(x, y) == Vec4b(0, 0, 0, 0));
}

TEST_CASE("FramebufferEntt::render scale_を縮小すると貼り戻し範囲外はclear_original_により透明になる") {
  auto comp   = make_test_comp(60, 60);
  auto target = make_target(60, 60);
  target->fill_rgba(Vec4b(255, 0, 0, 255)); // 全面赤
  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = true;
  fb->scale_          = Vec2(50, 50); // 半分に縮小して中央に貼り戻す
  CHECK(fb->render(comp.get(), target.get(), 0));

  CHECK(target->rgba(29, 29) == Vec4b(255, 0, 0, 255)); // 縮小後の範囲内(中央)は元の赤が残る
  CHECK(target->rgba(1, 1) == Vec4b(0, 0, 0, 0));       // 縮小後の範囲外はクリアされ透明
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
  fb->alpha_          = 0; // 貼り戻しを無効化し、clear_original_の効果だけを見る
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

namespace {

// 左半分に四角(中心20,20)・丸(中心20,45)・動画を配置し、色調補正(明るさ50%)付きフレームバッファをW/2だけ右へずらして貼り戻す
Ref<Composition> build_scene(bool clear_original, Ref<FramebufferEntt>* fb_out) {
  constexpr int W = 120, H = 60;
  auto comp = cutil::make_ref<Composition>("scene", W, H, 30);

  auto movie = Movie::Create("bg_movie", "../assets/movies/big_buck_bunny_360_10s.mp4");
  REQUIRE(movie->get_input_plugin() != nullptr);
  movie->pos        = Vec3(-W / 4.0f, 0, 0); // 左半分寄りに配置
  movie->scale      = Vec2(20, 20);
  movie->trk.fstart = 0;
  movie->trk.fend   = 10;

  auto rect        = ShapeEntt::Create("rect", ShapeType_Rect);
  rect->pos_       = Vec3(10, 10, 0);
  rect->size_      = Vec2(20, 20);
  rect->color_     = Vec4b(200, 50, 50, 255);
  rect->trk.fstart = 0;
  rect->trk.fend   = 10;

  auto circ        = ShapeEntt::Create("circ", ShapeType_Circle);
  circ->pos_       = Vec3(10, 35, 0);
  circ->size_      = Vec2(20, 20);
  circ->color_     = Vec4b(50, 50, 200, 255);
  circ->trk.fstart = 0;
  circ->trk.fend   = 10;

  detail::register_default_filters();
  detail::activate_all_plugins();
  auto* filters                = &detail::AppMain::Get()->filters;
  FilterPluginTable* color_plg = nullptr;
  for(auto& f : *filters)
    if(std::string(f.name.c_str()) == "色調補正") color_plg = &f;
  REQUIRE(color_plg != nullptr);

  auto fb             = FramebufferEntt::Create("fb");
  fb->clear_original_ = clear_original;
  fb->pos_            = Vec3((float)W / 2.0f, 0, 0); // Composition幅の半分だけ右へ
  fb->trk.fstart      = 0;
  fb->trk.fend        = 10;

  TrackObject::FilterParam fp;
  fp.plg_ = color_plg;
  cutil::Prop custom_defaults;
  custom_defaults.set<float>("brightness", 50.0f); // 明るさ50%
  custom_defaults.set<float>("contrast", 100.0f);
  fp.props.add_props(custom_defaults);
  fp.enabled = true;
  fb->trk.filters.push_back(fp);

  comp->insert_entity(movie, 0);
  comp->insert_entity(rect, 1);
  comp->insert_entity(circ, 2);
  comp->insert_entity(fb, 3);

  if(fb_out) *fb_out = fb;
  return comp;
}

} // namespace

TEST_CASE("FramebufferEntt: 位置シフト+色調補正で暗い複製が右半分に現れる(clear_original_=false)") {
  Ref<FramebufferEntt> fb;
  auto comp = build_scene(false, &fb);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp.get(), 0, out));

  // 左半分: 元の四角/丸がそのままの明るさで残っている
  CHECK(out->rgba(20, 20) == Vec4b(200, 50, 50, 255)); // 四角(原寸)
  CHECK(out->rgba(20, 45) == Vec4b(50, 50, 200, 255)); // 丸(原寸)

  // 右半分: フレームバッファ経由の複製が明るさ50%で現れる
  CHECK(out->rgba(80, 20) == Vec4b(100, 25, 25, 255)); // 四角(暗い複製)
  CHECK(out->rgba(80, 45) == Vec4b(25, 25, 100, 255)); // 丸(暗い複製)
}

TEST_CASE("FramebufferEntt: clear_original_=trueだと元の位置は消え、暗い複製が右半分に1つずつだけ残る") {
  Ref<FramebufferEntt> fb;
  auto comp = build_scene(true, &fb);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp.get(), 0, out));

  // 左半分: フレームバッファでクリアされ、元の四角/丸は消えている
  CHECK(out->rgba(20, 20) == Vec4b(0, 0, 0, 0));
  CHECK(out->rgba(20, 45) == Vec4b(0, 0, 0, 0));

  // 右半分: 暗い複製だけが残る
  CHECK(out->rgba(80, 20) == Vec4b(100, 25, 25, 255)); // 四角(暗い複製)
  CHECK(out->rgba(80, 45) == Vec4b(25, 25, 100, 255)); // 丸(暗い複製)
}
