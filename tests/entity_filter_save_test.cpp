#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>

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

  auto* filters = &detail::AppMain::Get()->filters;
  FilterPluginTable* color_correction = nullptr;
  for(auto& f : *filters)
    if(std::string(f.name.c_str()) == "色調補正") color_correction = &f;
  REQUIRE(color_correction != nullptr);

  TrackObject::FilterParam fp;
  fp.plg_ = color_correction;
  fp.props.add_props(color_correction->defaults);
  fp.props.set_value<float>(0, 0, 42.0f); // hue(先頭フィールド)を非デフォルト値に変更
  fp.enabled = false;
  img->trk.filters.push_back(fp);

  auto saved = img->getSaveProps();
  REQUIRE(saved.contains("filters"));
  auto filters_p = saved.get_child("filters");
  REQUIRE(filters_p.get<int32_t>("count") == 1);
  auto filter0 = filters_p.get_child("filter_0");
  REQUIRE(filter0.contains("params"));
  auto params = filter0.get_child("params");
  REQUIRE(params.contains("hue"));
  CHECK(params.get<float>("hue") == doctest::Approx(42.0f));

  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);

  CHECK(loaded->trk.filters.size() == 1);
  REQUIRE(loaded->trk.filters.size() == 1);
  CHECK(loaded->trk.filters[0].plg_ == color_correction);
  CHECK(loaded->trk.filters[0].enabled == false);
  CHECK(loaded->trk.filters[0].props.get<float>(0) == doctest::Approx(42.0f));
}

TEST_CASE("Entity::getSaveProps/fromSaveProps: フィルタが無ければ空のまま復元される") {
  ensure_filters_registered();
  Project::New();

  auto img = Image::Create("no_filter_save_test", 4, 4);
  REQUIRE(img != nullptr);

  auto saved  = img->getSaveProps();
  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);
  CHECK(loaded->trk.filters.empty());
}
