#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/render2d/renderer.hpp>

using namespace mu;

namespace {
// 100x100の透明背景コンポを作る(中心=(50,50))
Ref<Composition> make_comp() {
  auto comp      = cutil::make_ref<Composition>("group_test", 100, 100, 30);
  comp->bg_color = 0;
  return comp;
}

Ref<ShapeEntt> add_rect(Composition* comp, int layer, Vec3 pos) {
  auto s     = ShapeEntt::Create("rect", ShapeType_Rect);
  s->size_   = Vec2(10, 10);
  s->color_  = Vec4b(255, 0, 0, 255);
  s->pos_    = pos;
  s->fstart_ = 0;
  s->fend_   = 10;
  comp->insert_entity(s, layer);
  return s;
}

Ref<GroupEntt> add_group(Composition* comp, int layer, Vec3 pos) {
  auto g     = GroupEntt::Create("group");
  g->pos_    = pos;
  g->fstart_ = 0;
  g->fend_   = 10;
  comp->insert_entity(g, layer);
  return g;
}

Ref<Image> render(Composition* comp, int frame = 0) {
  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(comp, frame, out));
  return out;
}

bool red_at(const Ref<Image>& img, int x, int y) { return img->rgba(x, y)[3] > 0; }
} // namespace

TEST_CASE("グループ制御: 位置が下のレイヤーの子へ加算される") {
  auto comp = make_comp();
  add_group(comp.get(), 0, Vec3(20, 0, 0));
  add_rect(comp.get(), 1, Vec3(0, 0, 0));
  auto out = render(comp.get());
  CHECK(red_at(out, 70, 50));
  CHECK_FALSE(red_at(out, 50, 50));
}

TEST_CASE("グループ制御: 回転で子がグループ原点まわりに公転する") {
  auto comp    = make_comp();
  auto g       = add_group(comp.get(), 0, Vec3(0, 0, 0));
  g->rotation_ = 90.0f;
  add_rect(comp.get(), 1, Vec3(20, 0, 0));
  auto out = render(comp.get());
  CHECK(red_at(out, 50, 70)); // (20,0)を時計回り(Y下向き)に90度回すと(0,20)
  CHECK_FALSE(red_at(out, 70, 50));
}

TEST_CASE("グループ制御: 拡大率が子の位置にも掛かる") {
  auto comp = make_comp();
  auto g    = add_group(comp.get(), 0, Vec3(0, 0, 0));
  g->scale_ = 200.f;
  add_rect(comp.get(), 1, Vec3(10, 0, 0));
  auto out = render(comp.get());
  CHECK(red_at(out, 78, 50));       // 位置(10,0)が20pxへ拡大され、矩形も20px幅(60..80)になる
  CHECK_FALSE(red_at(out, 55, 50)); // 拡大されていなければ子は55..65に描かれる
}

TEST_CASE("グループ制御: 対象レイヤー数の範囲外や上のレイヤーには効かない") {
  auto comp         = make_comp();
  auto g            = add_group(comp.get(), 1, Vec3(20, 0, 0));
  g->target_layers_ = 1;
  add_rect(comp.get(), 0, Vec3(0, -30, 0)); // 上のレイヤー: 効かない
  add_rect(comp.get(), 2, Vec3(0, 0, 0));   // 範囲内
  add_rect(comp.get(), 3, Vec3(0, 30, 0));  // 範囲外
  auto out = render(comp.get());
  CHECK(red_at(out, 50, 20));
  CHECK_FALSE(red_at(out, 70, 20));
  CHECK(red_at(out, 70, 50));
  CHECK(red_at(out, 50, 80));
  CHECK_FALSE(red_at(out, 70, 80));
}

TEST_CASE("グループ制御: 対象レイヤー数0なら以降のすべてのレイヤーに効く") {
  auto comp = make_comp();
  add_group(comp.get(), 0, Vec3(20, 0, 0));
  add_rect(comp.get(), 3, Vec3(0, 0, 0));
  auto out = render(comp.get());
  CHECK(red_at(out, 70, 50));
}

TEST_CASE("グループ制御: 入れ子のグループは外側から合成される") {
  auto comp = make_comp();
  add_group(comp.get(), 0, Vec3(10, 0, 0));
  add_group(comp.get(), 1, Vec3(10, 0, 0));
  add_rect(comp.get(), 2, Vec3(0, 0, 0));
  auto out = render(comp.get());
  CHECK(red_at(out, 70, 50));
  CHECK_FALSE(red_at(out, 60, 50)); // 60はどちらか1段しか効かない場合の位置(矩形は65..75)
}

TEST_CASE("グループ制御: 不透明度が乗算される") {
  auto comp = make_comp();
  auto g    = add_group(comp.get(), 0, Vec3(0, 0, 0));
  g->alpha_ = 0.5f;
  auto r    = add_rect(comp.get(), 1, Vec3(0, 0, 0));
  r->alpha_ = 0.5f;
  auto out  = render(comp.get());
  CHECK(out->rgba(50, 50)[3] == doctest::Approx(64).epsilon(0.1));
}

TEST_CASE("グループ制御: 表示期間外のグループは効かない") {
  auto comp = make_comp();
  auto g    = add_group(comp.get(), 0, Vec3(20, 0, 0));
  g->fend_  = 3;
  add_rect(comp.get(), 1, Vec3(0, 0, 0));
  auto out = render(comp.get(), 5);
  CHECK(red_at(out, 50, 50));
  CHECK_FALSE(red_at(out, 70, 50));
}

TEST_CASE("グループ制御: 保存・復元で対象レイヤー数と変換が保たれる") {
  auto g            = GroupEntt::Create("g");
  g->target_layers_ = 3;
  g->pos_           = Vec3(5, 6, 0);
  auto restored     = Entity::fromSaveProps(g->getSaveProps());
  auto* rg          = dynamic_cast<GroupEntt*>(restored.get());
  REQUIRE(rg != nullptr);
  CHECK(rg->target_layers_ == 3);
  CHECK(rg->pos_ == g->pos_);
}
