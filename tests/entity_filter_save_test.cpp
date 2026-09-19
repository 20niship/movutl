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
  int pos_idx = img->anim_props_.index_of("pos_");
  REQUIRE(pos_idx >= 0);
  auto& clip = std::get<PAniClip<Vec3>>(img->anim_props_[pos_idx]);
  clip.add_keyframe(0, Vec3(0, 0, 0));
  clip.add_keyframe(10, Vec3(100, 0, 0));

  img->apply_animated_props(5);
  CHECK(img->getTransformProps().get<Vec3>("pos_")[0] == doctest::Approx(50.0f));

  auto saved  = img->getSaveProps();
  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);
  int loaded_idx = loaded->anim_props_.index_of("pos_");
  REQUIRE(loaded_idx >= 0);
  CHECK(loaded->anim_props_.has_animation(loaded_idx));
  loaded->apply_animated_props(10);
  CHECK(loaded->getTransformProps().get<Vec3>("pos_")[0] == doctest::Approx(100.0f));
}

TEST_CASE("Entity::collect_animated_frames/move_keyframes_at/erase_keyframes_at: 本体+フィルタ横断の集約操作") {
  ensure_filters_registered();
  Project::New();

  auto img = Image::Create("aggregate_anim_test", 4, 4);
  REQUIRE(img != nullptr);
  img->fstart_ = 0;
  img->fend_   = 50;

  img->ensure_anim_props();
  int pos_idx   = img->anim_props_.index_of("pos_");
  int alpha_idx = img->anim_props_.index_of("alpha_");
  REQUIRE(pos_idx >= 0);
  REQUIRE(alpha_idx >= 0);
  auto& pos_clip = std::get<PAniClip<Vec3>>(img->anim_props_[pos_idx]);
  pos_clip.add_keyframe(0, Vec3(0, 0, 0));
  pos_clip.add_keyframe(20, Vec3(100, 0, 0)); // 同じframe(20)にalphaもキーを打つ
  auto& alpha_clip = std::get<PAniClip<float>>(img->anim_props_[alpha_idx]);
  alpha_clip.add_keyframe(0, 1.0f);
  alpha_clip.add_keyframe(20, 0.0f);

  auto* filters                       = &detail::AppMain::Get()->filters;
  FilterPluginTable* color_correction = nullptr;
  for(auto& f : *filters)
    if(std::string(f.name.c_str()) == "色調補正") color_correction = &f;
  REQUIRE(color_correction != nullptr);
  FilterParam fp;
  fp.plg_ = color_correction;
  fp.props.add_props(color_correction->defaults);
  img->filters_.push_back(fp);
  auto& hue_clip = std::get<PAniClip<float>>(img->filters_.back().props[0]); // filters_へのpush_backはコピーなので、参照は格納後のものを取る
  hue_clip.add_keyframe(0, 0.0f);
  hue_clip.add_keyframe(20, 90.0f); // フィルタパラメータにも同じframe(20)でキー

  auto frames = img->collect_animated_frames();
  REQUIRE(frames.size() == 2);
  CHECK(frames[0] == 0);
  CHECK(frames[1] == 20);

  REQUIRE(img->move_keyframes_at(20, 30));
  CHECK_FALSE(pos_clip.has_key_at(20));
  CHECK(pos_clip.has_key_at(30));
  CHECK_FALSE(alpha_clip.has_key_at(20));
  CHECK(alpha_clip.has_key_at(30));
  CHECK_FALSE(hue_clip.has_key_at(20));
  CHECK(hue_clip.has_key_at(30));

  REQUIRE(img->erase_keyframes_at(30));
  CHECK_FALSE(pos_clip.has_key_at(30));
  CHECK_FALSE(alpha_clip.has_key_at(30));
  CHECK_FALSE(hue_clip.has_key_at(30));
  // 各clipとも frame=0 の1個目のキーは残るが、単一キーはアニメーションではないため中間点として集計されない
  CHECK(img->collect_animated_frames().empty());
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

TEST_CASE("Entity::ensure_anim_props: 文字列プロパティはanim_props_に含まれない(apply_animated_propsで入力がリセットされない)") {
  Project::New();
  auto img = Image::Create("anim_string_test", 4, 4);
  REQUIRE(img != nullptr);
  img->ensure_anim_props();
  const auto* info = img->getPropsInfo();
  REQUIRE(info != nullptr);
  for(const auto& f : info->fields) {
    if(f.type == cutil::prop_info_of<std::string>()) CHECK(img->anim_props_.index_of(f.name) < 0);
  }
}

TEST_CASE("Entity: 中間点はトラック開始(fstart_)からの相対frameで評価/集約/移動される") {
  Project::New();
  auto img = Image::Create("anim_rel_test", 4, 4);
  REQUIRE(img != nullptr);
  img->fstart_ = 50;
  img->fend_   = 200;
  img->ensure_anim_props();
  int idx = img->anim_props_.index_of("alpha_");
  REQUIRE(idx >= 0);
  img->anim_props_.add_keyframe<float>(idx, 0, 0.0f);
  img->anim_props_.add_keyframe<float>(idx, 100, 1.0f);

  img->apply_animated_props(100); // 絶対100 = 相対50
  CHECK(img->alpha_ == doctest::Approx(0.5f));
  img->apply_animated_props(10); // 開始より前は相対0
  CHECK(img->alpha_ == doctest::Approx(0.0f));

  auto frames = img->collect_animated_frames(); // コンポジション絶対frameで返る
  REQUIRE(frames.size() == 2);
  CHECK(frames[0] == 50);
  CHECK(frames[1] == 150);

  CHECK(img->move_keyframes_at(150, 160));
  CHECK(img->anim_props_.has_key_at(idx, 110));
  CHECK(img->erase_keyframes_at(160));
  CHECK_FALSE(img->anim_props_.has_key_at(idx, 110));
}

TEST_CASE("Entity::on_len_change_done: split相当の長さ変更で範囲外の中間点が境界値へ整理される") {
  Project::New();
  auto img = Image::Create("len_change_test", 4, 4);
  REQUIRE(img != nullptr);
  img->fstart_ = 0;
  img->fend_   = 100;
  img->ensure_anim_props();
  int idx = img->anim_props_.index_of("alpha_");
  img->anim_props_.add_keyframe<float>(idx, 0, 0.0f);
  img->anim_props_.add_keyframe<float>(idx, 100, 1.0f);

  img->fstart_ = 40; // 後半側(frame40で分割)
  img->on_len_change_done(0);
  CHECK(img->anim_props_.get<float>(idx, 0) == doctest::Approx(0.4f));
  CHECK(img->anim_props_.get<float>(idx, 60) == doctest::Approx(1.0f));

  img->fend_ = 70; // さらに末尾を切り詰める(長さ30)
  img->on_len_change_done(40);
  CHECK(img->anim_props_.get<float>(idx, 30) == doctest::Approx(0.7f));
  CHECK(img->anim_props_.keyframe_frames(idx).back() == 30);
}
