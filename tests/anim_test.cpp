#include <doctest/doctest.h>
#include <movutl/core/anim.hpp>

using namespace mu;

TEST_CASE("PAniClip::add_keyframe: frame昇順を保って挿入され、既存frameは値を上書きする") {
  PAniClip<float> clip;
  clip.clear();
  clip.add_keyframe(10, 100.0f);
  clip.add_keyframe(0, 0.0f);
  clip.add_keyframe(20, 200.0f);
  REQUIRE(clip.keys.size() == 3);
  CHECK(clip.keys[0].frame_ == 0);
  CHECK(clip.keys[1].frame_ == 10);
  CHECK(clip.keys[2].frame_ == 20);

  clip.add_keyframe(10, 150.0f);
  REQUIRE(clip.keys.size() == 3);
  CHECK(clip.keys[1].value_ == doctest::Approx(150.0f));
}

TEST_CASE("PAniClip<float>::get: キーフレーム間を線形補間する") {
  PAniClip<float> clip;
  clip.clear();
  clip.add_keyframe(0, 0.0f, AniInterpType::LINEAR);
  clip.add_keyframe(10, 100.0f, AniInterpType::LINEAR);

  CHECK(clip.get(0) == doctest::Approx(0.0f));
  CHECK(clip.get(5) == doctest::Approx(50.0f));
  CHECK(clip.get(10) == doctest::Approx(100.0f));
}

TEST_CASE("PAniClip::get: 範囲外frameは端のキーフレーム値でクランプされる") {
  PAniClip<float> clip;
  clip.clear();
  clip.add_keyframe(10, 10.0f);
  clip.add_keyframe(20, 20.0f);

  CHECK(clip.get(0) == doctest::Approx(10.0f));
  CHECK(clip.get(100) == doctest::Approx(20.0f));
}

TEST_CASE("PAniClip<Vec2>::get: ベクトル型も補間できる") {
  PAniClip<Vec2> clip;
  clip.clear();
  clip.add_keyframe(0, Vec2(0, 0));
  clip.add_keyframe(10, Vec2(100, 200));

  auto v = clip.get(5);
  CHECK(v[0] == doctest::Approx(50.0f));
  CHECK(v[1] == doctest::Approx(100.0f));
}

TEST_CASE("PAniClip<bool>::get: 補間できない型はステップ(区間開始側の値)を返す") {
  PAniClip<bool> clip;
  clip.clear();
  clip.add_keyframe(0, false);
  clip.add_keyframe(10, true);

  CHECK(clip.get(0) == false);
  CHECK(clip.get(5) == false);
  CHECK(clip.get(10) == true);
}

TEST_CASE("AnimProps::set_value: 単一キーのみの場合は値を直接書き換える(アニメーション化しない)") {
  AnimProps props;
  props.add_prop<float>("x", 1.0f);
  props.set_value<float>(0, 50, 5.0f);

  auto& clip = std::get<PAniClip<float>>(props[0]);
  REQUIRE(clip.keys.size() == 1);
  CHECK(clip.keys[0].value_ == doctest::Approx(5.0f));
}

TEST_CASE("AnimProps::set_value: 既にアニメーションしている場合はキーフレームを追加する") {
  AnimProps props;
  props.add_prop<float>("x", 0.0f);
  auto& clip0 = std::get<PAniClip<float>>(props[0]);
  clip0.add_keyframe(10, 10.0f); // 2キー目を追加してhas_animation()==trueにする

  props.set_value<float>(0, 20, 20.0f);
  auto& clip = std::get<PAniClip<float>>(props[0]);
  REQUIRE(clip.keys.size() == 3);
  CHECK(clip.keys[2].frame_ == 20);
  CHECK(clip.keys[2].value_ == doctest::Approx(20.0f));
}

TEST_CASE("AnimProps::get(frame): frameに応じて補間された値がcutil::Propへ反映される") {
  AnimProps props;
  props.add_prop<float>("x", 0.0f);
  auto& clip = std::get<PAniClip<float>>(props[0]);
  clip.add_keyframe(0, 0.0f);
  clip.add_keyframe(10, 100.0f);

  auto p_mid = props.get(5);
  CHECK(p_mid.get<float>("x") == doctest::Approx(50.0f));

  auto p_end = props.get(10);
  CHECK(p_end.get<float>("x") == doctest::Approx(100.0f));
}
