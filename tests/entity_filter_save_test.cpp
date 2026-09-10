#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/anim.hpp>

using namespace mu;

namespace {
void ensure_filters_registered() {
  static bool once = [] {
    if(detail::AppMain::Get()->filters.empty()) detail::register_default_filters();
    detail::activate_all_plugins(); // fn_init()を呼びFilterPluginTable::defaultsを構築する
    return true;
  }();
  (void)once;
}
} // namespace

TEST_CASE("Entity::getSaveProps/fromSaveProps: フィルタ(enabled/パラメータ)が保存/復元される") {
  ensure_filters_registered();
  Project::New();

  auto img = Image::Create("filter_save_test", 4, 4);
  REQUIRE(img != nullptr);

  auto* filters                       = &detail::AppMain::Get()->filters;
  FilterPluginTable* color_correction = nullptr;
  for(auto& f : *filters)
    if(std::string(f.name.c_str()) == "色調補正") color_correction = &f;
  REQUIRE(color_correction != nullptr);

  FilterParam fp;
  fp.plg_ = color_correction;
  fp.props.add_props(color_correction->defaults);
  fp.props.set_value<float>(0, 0, 42.0f); // hue(先頭フィールド)を非デフォルト値に変更
  fp.enabled = false;
  img->filters_.push_back(fp);

  auto saved = img->getSaveProps();
  REQUIRE(saved.contains("filters"));
  auto filters_p = saved.get_child("filters");
  REQUIRE(filters_p.get<int32_t>("count") == 1);
  auto filter0 = filters_p.get_child("filter_0");
  REQUIRE(filter0.contains("anim_params"));

  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);

  CHECK(loaded->filters_.size() == 1);
  REQUIRE(loaded->filters_.size() == 1);
  CHECK(loaded->filters_[0].plg_ == color_correction);
  CHECK(loaded->filters_[0].enabled == false);
  CHECK(loaded->filters_[0].props.get<float>(0) == doctest::Approx(42.0f));
}

TEST_CASE("Entity::anim_props_: 位置(Vec3)の中間点アニメーションがrender前評価/保存復元できる") {
  ensure_filters_registered();
  Project::New();

  auto img = Image::Create("entity_anim_test", 4, 4);
  REQUIRE(img != nullptr);
  img->fstart_ = 0;
  img->fend_   = 20;

  img->ensure_anim_props();
  int pos_idx = img->anim_props_.index_of("pos");
  REQUIRE(pos_idx >= 0);
  auto& clip = std::get<PAniClip<Vec3>>(img->anim_props_[pos_idx]);
  clip.add_keyframe(0, Vec3(0, 0, 0));
  clip.add_keyframe(10, Vec3(100, 0, 0));

  img->apply_animated_props(5);
  CHECK(img->getProps().get<Vec3>("pos")[0] == doctest::Approx(50.0f));

  auto saved  = img->getSaveProps();
  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);
  int loaded_idx = loaded->anim_props_.index_of("pos");
  REQUIRE(loaded_idx >= 0);
  CHECK(loaded->anim_props_.has_animation(loaded_idx));
  loaded->apply_animated_props(10);
  CHECK(loaded->getProps().get<Vec3>("pos")[0] == doctest::Approx(100.0f));
}

TEST_CASE("Entity::getSaveProps/fromSaveProps: フィルタが無ければ空のまま復元される") {
  ensure_filters_registered();
  Project::New();

  auto img = Image::Create("no_filter_save_test", 4, 4);
  REQUIRE(img != nullptr);

  auto saved  = img->getSaveProps();
  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);
  CHECK(loaded->filters_.empty());
}
