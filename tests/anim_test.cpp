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

TEST_CASE("PAniClip::has_key_at/erase_keyframe/move_keyframe: 中間点の追加/削除/移動") {
  PAniClip<float> clip;
  clip.clear();
  clip.add_keyframe(0, 0.0f);
  clip.add_keyframe(10, 100.0f);

  CHECK(clip.has_key_at(0));
  CHECK(clip.has_key_at(10));
  CHECK_FALSE(clip.has_key_at(5));

  CHECK(clip.move_keyframe(10, 20));
  CHECK(clip.has_key_at(20));
  CHECK_FALSE(clip.has_key_at(10));

  CHECK(clip.erase_keyframe(20));
  REQUIRE(clip.keys.size() == 1);
  CHECK_FALSE(clip.erase_keyframe(0)); // 最後の1個は削除できない
}

TEST_CASE("AnimProps::save/load_keys: キーフレーム列を保存し名前一致で復元する") {
  AnimProps src;
  src.add_prop<float>("x", 0.0f);
  src.add_prop<bool>("visible", true);
  auto& clip = std::get<PAniClip<float>>(src[0]);
  clip.add_keyframe(0, 1.0f);
  clip.add_keyframe(10, 100.0f);
  clip.add_keyframe(20, 50.0f);

  auto saved = src.save();

  AnimProps dst;
  dst.add_prop<float>("x", -1.0f);
  dst.add_prop<bool>("visible", false);
  dst.load_keys(saved);

  CHECK(dst.get<float>(0, 0) == doctest::Approx(1.0f));
  CHECK(dst.get<float>(0, 10) == doctest::Approx(100.0f));
  CHECK(dst.get<float>(0, 20) == doctest::Approx(50.0f));
  CHECK(dst.has_animation(0));
}
