#include <doctest/doctest.h>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/command.hpp>

using namespace mu;

TEST_CASE("Project::AddComposition") {
  Project::New();
  auto* cmp = Project::AddComposition("Sub", 640, 480, 24);
  REQUIRE(cmp != nullptr);
  CHECK(std::string(cmp->name.c_str()) == "Sub");
  CHECK(cmp->size[0] == 640);
  CHECK(cmp->size[1] == 480);
  CHECK(cmp->framerate == 24.0f);
  CHECK((cmp->flag & Composition::setting_dialog) != 0); // 新規追加時は設定ウィンドウが自動で開く
}

TEST_CASE("SplitCommand: 選択中クリップを現在フレームで分割する") {
  Project::New();
  detail::register_default_commands();
  auto* cmp = Project::GetActiveCompo();
  REQUIRE(cmp != nullptr);

  auto img = Image::Create("clip", 64, 64);
  REQUIRE(img != nullptr);
  img->trk.fstart = 0;
  img->trk.fend   = 100;
  cmp->insert_entity(img);
  cmp->frame = 40;

  clear_selected_entts();
  select_entt(img);

  CHECK(run_command("split"));

  CHECK(img->trk.fend == 40); // 前半は現在フレームまで短縮される

  bool found_second_half = false;
  for(auto& layer : cmp->layers) {
    for(auto& e : layer.entts) {
      if(e && e.get() != img.get() && e->trk.fstart == 40 && e->trk.fend == 100) found_second_half = true;
    }
  }
  CHECK(found_second_half);
}

TEST_CASE("duplicate_asset: Entityを複製できる") {
  Project::New();
  auto img = Image::Create("clip", 64, 64);
  REQUIRE(img != nullptr);
  img->trk.fstart = 10;
  img->trk.fend   = 90;

  auto clone = duplicate_asset(img);
  REQUIRE(clone != nullptr);
  CHECK(clone.get() != img.get());
  CHECK(clone->getType() == img->getType());
  CHECK(std::string(clone->name.c_str()) == std::string(img->name.c_str()));
  CHECK(clone->guid_ != img->guid_);

  CHECK(duplicate_asset(nullptr) == nullptr);
}

TEST_CASE("SplitCommand: 範囲外フレームでは分割しない") {
  Project::New();
  detail::register_default_commands();
  auto* cmp = Project::GetActiveCompo();
  REQUIRE(cmp != nullptr);

  auto img = Image::Create("clip2", 64, 64);
  REQUIRE(img != nullptr);
  img->trk.fstart = 0;
  img->trk.fend   = 100;
  cmp->insert_entity(img);
  cmp->frame = 200; // クリップ範囲外

  clear_selected_entts();
  select_entt(img);

  CHECK(run_command("split"));
  CHECK(img->trk.fend == 100); // 変化なし
}
