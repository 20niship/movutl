#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
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
