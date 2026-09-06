#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;

namespace {
// テスト用の一時フォルダへ.objファイルを書き出し、register_custom_objects()経由(実際のフォルダ自動スキャン)で登録する
void register_test_object(const std::string& text) {
  static int counter = 0;
  auto dir           = std::filesystem::temp_directory_path() / ("movutl_custom_object_test_" + std::to_string(counter++));
  std::filesystem::create_directories(dir);
  std::ofstream ofs(dir / "test.obj");
  ofs << text;
  ofs.close();

  auto saved                     = Config::Get()->lua_script_dirs;
  Config::Get()->lua_script_dirs = {dir.string()};
  detail::register_custom_objects();
  Config::Get()->lua_script_dirs = saved;
}
} // namespace

TEST_CASE("register_custom_objects: フォルダスキャン経由でカスタムオブジェクトが登録される") {
  std::string text = "--track0:太さ,1,10,2,1\n"
                     "@テストオブジェクト\n"
                     "obj.line(0, 0, 5, 0, 255, 255, 255, 255, obj.track0)\n";
  register_test_object(text);

  bool found = false;
  for(const auto& e : CustomObjectRegistry::Get()->list())
    if(e.name == "テストオブジェクト") found = true;
  CHECK(found);
  CHECK(CustomObjectRegistry::Get()->find_def("テストオブジェクト") != nullptr);
}

TEST_CASE("CustomObjectEntt::Create/render: インスタンス生成と描画ができる") {
  std::string text = "@線オブジェクト\n"
                     "obj.line(-5, 0, 5, 0, 255, 0, 0, 255, 1)\n";
  register_test_object(text);

  auto e = CustomObjectEntt::Create("線", "線オブジェクト");
  REQUIRE(e != nullptr);
  CHECK(e->getType() == EntityType_Custom);

  auto comp   = cutil::make_ref<Composition>("test", 20, 20, 30);
  auto target = cutil::make_ref<Image>();
  CHECK(e->render(comp.get(), target.get(), 0));
  CHECK(target->width == 20);
  CHECK(target->height == 20);
  CHECK(target->rgba(10, 10)[0] == 255); // 中心付近に赤い線が描かれているはず
}

TEST_CASE("CustomObjectEntt::Create: 未登録スクリプト名はnullptrを返す") {
  auto e = CustomObjectEntt::Create("なし", "存在しないスクリプト名");
  CHECK(e == nullptr);
}

TEST_CASE("Entity::getSaveProps/fromSaveProps: CustomObjectEnttのscript_name/paramsが保存/復元される") {
  std::string text = "--track0:太さ,1,10,2,1\n"
                     "@保存テストオブジェクト\n"
                     "obj.line(0, 0, 5, 0, 255, 255, 255, 255, obj.track0)\n";
  register_test_object(text);

  auto e = CustomObjectEntt::Create("保存テスト", "保存テストオブジェクト");
  REQUIRE(e != nullptr);
  e->params_.set<float>("太さ", 7.0f);

  auto saved  = e->getSaveProps();
  auto loaded = Entity::fromSaveProps(saved);
  REQUIRE(loaded != nullptr);
  CHECK(loaded->getType() == EntityType_Custom);

  auto* custom = dynamic_cast<CustomObjectEntt*>(loaded.get());
  REQUIRE(custom != nullptr);
  CHECK(custom->script_name_ == "保存テストオブジェクト");
  CHECK(custom->params_.get<float>("太さ") == doctest::Approx(7.0f));
}

TEST_CASE("集中線.obj: 実物のサンプルスクリプトが中心から放射状に線を描画する") {
  // plugins/scripts/集中線.obj(リポジトリに同梱の実物)をtestバイナリのカレントディレクトリ(build/)から相対参照する
  auto saved                     = Config::Get()->lua_script_dirs;
  Config::Get()->lua_script_dirs = {"../plugins/scripts"};
  detail::register_custom_objects();
  Config::Get()->lua_script_dirs = saved;

  REQUIRE(CustomObjectRegistry::Get()->find_def("集中線") != nullptr);

  auto e = CustomObjectEntt::Create("集中線テスト", "集中線");
  REQUIRE(e != nullptr);

  auto comp   = cutil::make_ref<Composition>("test", 200, 200, 30);
  auto target = cutil::make_ref<Image>();
  REQUIRE(e->render(comp.get(), target.get(), 0));
  CHECK(target->width == 200);
  CHECK(target->height == 200);

  // 既定パラメータ(本数=100,太さ=3,抜き=0)では中心から離れた複数方向に不透明ピクセルが描かれるはず
  int drawn_count = 0;
  for(size_t y = 0; y < target->height; y++)
    for(size_t x = 0; x < target->width; x++)
      if(target->rgba(x, y)[3] > 0) drawn_count++;
  CHECK(drawn_count > 100);

  // 中心そのものは抜き=0でも半径0地点なのでほぼ確実に線が重なって描かれている
  CHECK(target->rgba(100, 100)[3] > 0);
}

TEST_CASE("集中線.obj: パラメータ(本数)を変えると描画されるピクセル数が変化する") {
  auto saved                     = Config::Get()->lua_script_dirs;
  Config::Get()->lua_script_dirs = {"../plugins/scripts"};
  detail::register_custom_objects();
  Config::Get()->lua_script_dirs = saved;

  auto comp = cutil::make_ref<Composition>("test", 200, 200, 30);

  auto few = CustomObjectEntt::Create("few", "集中線");
  REQUIRE(few != nullptr);
  few->params_.set<float>("本数", 2.0f);
  few->params_.set<float>("太さ", 1.0f);
  auto few_img = cutil::make_ref<Image>();
  REQUIRE(few->render(comp.get(), few_img.get(), 0));

  auto many = CustomObjectEntt::Create("many", "集中線");
  REQUIRE(many != nullptr);
  many->params_.set<float>("本数", 200.0f);
  many->params_.set<float>("太さ", 1.0f);
  auto many_img = cutil::make_ref<Image>();
  REQUIRE(many->render(comp.get(), many_img.get(), 0));

  auto count_drawn = [](const Ref<Image>& img) {
    int n = 0;
    for(size_t y = 0; y < img->height; y++)
      for(size_t x = 0; x < img->width; x++)
        if(img->rgba(x, y)[3] > 0) n++;
    return n;
  };
  CHECK(count_drawn(many_img) > count_drawn(few_img));
}

TEST_CASE("Project::Save/Load: カスタムオブジェクトを含むプロジェクトがファイルへ保存・復元できる") {
  auto saved                     = Config::Get()->lua_script_dirs;
  Config::Get()->lua_script_dirs = {"../plugins/scripts"};
  detail::register_custom_objects();
  Config::Get()->lua_script_dirs = saved;

  Project::New(200, 200, 30);
  auto* comp = Composition::GetActiveComp();
  REQUIRE(comp != nullptr);

  auto e = CustomObjectEntt::Create("集中線レイヤー", "集中線");
  REQUIRE(e != nullptr);
  e->fstart_ = 0;
  e->fend_   = 50;
  e->params_.set<float>("本数", 42.0f);
  comp->insert_entity(e);

  auto tmp_path = (std::filesystem::temp_directory_path() / "movutl_custom_object_project_test.mvutl").string();
  Project::Save(tmp_path.c_str());

  Project::Load(tmp_path.c_str());
  auto* pj = Project::Get();
  REQUIRE(pj->entities.size() == 1);

  auto* loaded = dynamic_cast<CustomObjectEntt*>(pj->entities[0].get());
  REQUIRE(loaded != nullptr);
  CHECK(loaded->script_name_ == "集中線");
  CHECK(loaded->params_.get<float>("本数") == doctest::Approx(42.0f));
  CHECK(loaded->fstart_ == 0);
  CHECK(loaded->fend_ == 50);

  // レイヤー構成にも同じEntityが結び付けられて復元されている(guid経由の紐付け)
  REQUIRE(pj->compos_.size() == 1);
  bool found_in_layer = false;
  for(auto& layer : pj->compos_[0]->layers)
    for(auto& le : layer.entts)
      if(le.get() == loaded) found_in_layer = true;
  CHECK(found_in_layer);

  // 復元後も実際にレンダリングできる(def_の再解決が正しく行われている)ことを確認する
  auto target = cutil::make_ref<Image>();
  CHECK(loaded->render(pj->compos_[0].get(), target.get(), 0));

  std::filesystem::remove(tmp_path);
}
