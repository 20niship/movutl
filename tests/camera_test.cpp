#include <doctest/doctest.h>
#include <movutl/asset/camera.hpp>

using namespace mu;

TEST_CASE("カメラ制御: 既定カメラ(Z=-1024)は等倍で恒等変換") {
  auto c = Camera3D::Create("cam");
  auto x = c->view_xform();
  CHECK(x.scale == doctest::Approx(100.f));
  CHECK(x.pos[0] == doctest::Approx(0.f));
  CHECK(x.pos[1] == doctest::Approx(0.f));
  CHECK(x.rotation == doctest::Approx(0.f));
}

TEST_CASE("カメラ制御: カメラを右へ動かすと描画が左へ動く/近づくと拡大される") {
  auto c  = Camera3D::Create("cam");
  c->pos_ = Vec3(100, 0, -512);
  auto x  = c->view_xform();
  CHECK(x.scale == doctest::Approx(200.f)); // 1024/512
  CHECK(x.pos[0] == doctest::Approx(-200.f));
  auto w = x.compose(GroupXform{Vec3(100, 0, 0), 100.f, 0.f, 1.f});
  CHECK(w.pos[0] == doctest::Approx(0.f)); // カメラ真正面のオブジェクトは画面中央
}

TEST_CASE("カメラ制御: 傾きは逆回転として効く") {
  auto c       = Camera3D::Create("cam");
  c->rotation_ = 30.f;
  CHECK(c->view_xform().rotation == doctest::Approx(-30.f));
}

TEST_CASE("カメラ制御: affectsは自身より下の対象レイヤー数まで") {
  auto c = Camera3D::Create("cam");
  CHECK(c->affects(2, 3));
  CHECK_FALSE(c->affects(2, 2));
  CHECK_FALSE(c->affects(2, 1));
  c->target_layers_ = 2;
  CHECK(c->affects(2, 4));
  CHECK_FALSE(c->affects(2, 5));
}

TEST_CASE("カメラ制御: 保存・復元で位置と対象レイヤー数が保たれる") {
  auto c            = Camera3D::Create("cam");
  c->pos_           = Vec3(1, 2, -300);
  c->target_layers_ = 4;
  auto restored     = Entity::fromSaveProps(c->getSaveProps());
  auto* rc          = dynamic_cast<Camera3D*>(restored.get());
  REQUIRE(rc != nullptr);
  CHECK(rc->pos_ == c->pos_);
  CHECK(rc->target_layers_ == 4);
}
