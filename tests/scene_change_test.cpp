#include <doctest/doctest.h>
#include <movutl/asset/scene_change.hpp>

using namespace mu;

TEST_CASE("シーンチェンジ: 進行度は区間内で0→1、区間外はクランプ、長さ0は1") {
  CHECK(SceneChangeProgress(10, 10, 20) == doctest::Approx(0.f));
  CHECK(SceneChangeProgress(15, 10, 20) == doctest::Approx(0.5f));
  CHECK(SceneChangeProgress(20, 10, 20) == doctest::Approx(1.f));
  CHECK(SceneChangeProgress(0, 10, 20) == doctest::Approx(0.f));
  CHECK(SceneChangeProgress(99, 10, 20) == doctest::Approx(1.f));
  CHECK(SceneChangeProgress(5, 10, 10) == doctest::Approx(1.f));
}

TEST_CASE("シーンチェンジ: フェードの重みは位置に依らずp") { CHECK(SceneChangeWeight(SceneChangeType_Fade, 0.3f, 0.1f, 0.9f, false, 0.f) == doctest::Approx(0.3f)); }

TEST_CASE("シーンチェンジ: 左右ワイプは境界より左が遷移後、反転で右から") {
  CHECK(SceneChangeWeight(SceneChangeType_WipeLR, 0.5f, 0.25f, 0.5f, false, 0.f) == 1.f);
  CHECK(SceneChangeWeight(SceneChangeType_WipeLR, 0.5f, 0.75f, 0.5f, false, 0.f) == 0.f);
  CHECK(SceneChangeWeight(SceneChangeType_WipeLR, 0.5f, 0.75f, 0.5f, true, 0.f) == 1.f);
  CHECK(SceneChangeWeight(SceneChangeType_WipeLR, 0.f, 0.f, 0.f, false, 0.f) == 0.f);   // p=0は全面A
  CHECK(SceneChangeWeight(SceneChangeType_WipeLR, 1.f, 0.99f, 0.f, false, 0.f) == 1.f); // p=1は全面B
}

TEST_CASE("シーンチェンジ: ぼかし付きでもp=0/1で全面A/全面B、境界は中間値") {
  for(int type : {SceneChangeType_WipeLR, SceneChangeType_WipeUD, SceneChangeType_Circle})
    for(float x : {0.f, 0.5f, 1.f}) {
      CHECK(SceneChangeWeight(type, 0.f, x, x, false, 0.3f) == doctest::Approx(0.f));
      CHECK(SceneChangeWeight(type, 1.f, x, x, false, 0.3f) == doctest::Approx(1.f));
    }
  float w = SceneChangeWeight(SceneChangeType_WipeUD, 0.5f, 0.5f, 0.5f, false, 0.2f);
  CHECK(w > 0.f);
  CHECK(w < 1.f);
}

TEST_CASE("シーンチェンジ: 円形ワイプは中心から広がる") {
  CHECK(SceneChangeWeight(SceneChangeType_Circle, 0.5f, 0.5f, 0.5f, false, 0.f) == 1.f);
  CHECK(SceneChangeWeight(SceneChangeType_Circle, 0.5f, 0.f, 0.f, false, 0.f) == 0.f);
}

TEST_CASE("シーンチェンジ: 直上の1レイヤーにだけ効く") {
  auto s = SceneChangeEntt::Create("sc");
  CHECK(s->affects(3, 4));
  CHECK_FALSE(s->affects(3, 3));
  CHECK_FALSE(s->affects(3, 5));
}

TEST_CASE("シーンチェンジ: exoの名前から種類と方向が決まる") {
  int t;
  bool inv;
  REQUIRE(SceneChangeFromExoName("クロスフェード", t, inv));
  CHECK(t == SceneChangeType_Fade);
  REQUIRE(SceneChangeFromExoName("右から左へワイプ", t, inv));
  CHECK(t == SceneChangeType_WipeLR);
  CHECK(inv);
  REQUIRE(SceneChangeFromExoName("円形ワイプ", t, inv));
  CHECK(t == SceneChangeType_Circle);
  CHECK_FALSE(SceneChangeFromExoName("動画ファイル", t, inv));
}

TEST_CASE("シーンチェンジ: 保存・復元で種類/反転/ぼかしが保たれる") {
  auto s        = SceneChangeEntt::Create("sc");
  s->type_      = SceneChangeType_WipeUD;
  s->invert_    = true;
  s->blur_      = 0.4f;
  auto restored = Entity::fromSaveProps(s->getSaveProps());
  auto* rs      = dynamic_cast<SceneChangeEntt*>(restored.get());
  REQUIRE(rs != nullptr);
  CHECK(rs->type_ == SceneChangeType_WipeUD);
  CHECK(rs->invert_);
  CHECK(rs->blur_ == doctest::Approx(0.4f));
}
