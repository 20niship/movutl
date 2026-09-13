#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <cstdio>
#include <doctest/doctest.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/binding/imgui_custom_values.hpp>
#include <movutl/binding/lua_command.hpp>
#include <movutl/binding/lua_ui_panel.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/filesystem.hpp>
#include <unordered_map>

using namespace mu;

namespace {
// CommandManagerがLuaRefを無期限保持するため、close後に同名コマンド再登録でuse-after-freeになる。テストではlua_close()しない(意図的リーク)
lua_State* make_test_lua() {
  lua_State* L = luaL_newstate();
  luaL_openlibs(L);
  detail::generated_lua_binding_movutl(L); // save_project/open_project/has_project_path等はpygen生成側のバインディング
  detail::generated_lua_binding_imgui(L);
  detail::binding_custom_vectors(L); // imgui.ImVec2等
  detail::bind_lua_command_api(L);
  detail::bind_lua_ui_panel_api(L);
  // widgets/utility_window.luaのrequire("icons_fontawesome6")解決用(init.luaが本来設定するpackage.pathをテストでも再現する)
  luaL_dostring(L, "package.path = package.path .. ';../lancher/runtime/?.lua'");
  return L;
}
} // namespace

TEST_CASE("shortcuts.lua + widgets/utility_window.lua: 5つのショートカットコマンドが登録され、shortcut文字列が期待通りになる") {
  lua_State* L = make_test_lua();
  REQUIRE(luaL_dofile(L, "../lancher/runtime/shortcuts.lua") == 0);
  REQUIRE(luaL_dofile(L, "../lancher/runtime/widgets/utility_window.lua") == 0); // add_object_menuコマンドはこちらに定義されている

  CHECK(has_command("add_object_menu"));
  CHECK(has_command("save_project_cmd"));
  CHECK(has_command("save_project_as_cmd"));
  CHECK(has_command("open_project_cmd"));
  CHECK(has_command("import_media_cmd"));

  std::unordered_map<std::string, std::string> shortcuts;
  for(const auto& info : get_command_infos()) shortcuts[info.id] = info.shortcut;
  CHECK(shortcuts["add_object_menu"] == "shift+a");
  CHECK(shortcuts["save_project_cmd"] == "ctrl+s");
  CHECK(shortcuts["save_project_as_cmd"] == "ctrl+shift+s");
  CHECK(shortcuts["open_project_cmd"] == "ctrl+o");
  CHECK(shortcuts["import_media_cmd"] == "ctrl+i");
}

TEST_CASE("LuaCommand: on_startの戻り値がCommandStatusへ正しく変換される") {
  lua_State* L = make_test_lua();

  const char* script = R"(
    movutl.register_command("lua_cmd_finished_test", "", "", "", {
      on_start = function(self) return nil end,
    })
    movutl.register_command("lua_cmd_failed_test", "", "", "", {
      on_start = function(self) return "failed" end,
    })
    _G.lua_cmd_running_ticks = 0
    movutl.register_command("lua_cmd_running_test", "", "", "", {
      on_start = function(self) return "running" end,
      tick = function(self)
        _G.lua_cmd_running_ticks = _G.lua_cmd_running_ticks + 1
        if _G.lua_cmd_running_ticks >= 2 then return nil end
        return "running"
      end,
    })
  )";
  REQUIRE(luaL_dostring(L, script) == 0);

  CHECK(run_command("lua_cmd_finished_test"));     // nil返却 -> Finished扱い(成功)
  CHECK_FALSE(run_command("lua_cmd_failed_test")); // "failed" -> Failed扱い(run_commandはfalseを返す)
  CHECK(run_command("lua_cmd_running_test"));      // "running" -> Running扱い(run_commandはtrueを返す)

  // Running中はtick_running_commands()の度にLua側のtick()が呼ばれる
  tick_running_commands();
  lua_getglobal(L, "lua_cmd_running_ticks");
  CHECK(lua_tointeger(L, -1) == 1);
  lua_pop(L, 1);

  tick_running_commands(); // 2回目のtickでnil("Finished")を返し実行中リストから外れる
  lua_getglobal(L, "lua_cmd_running_ticks");
  CHECK(lua_tointeger(L, -1) == 2);
  lua_pop(L, 1);

  tick_running_commands(); // 既に完了済みなのでこれ以上ticksは増えない
  lua_getglobal(L, "lua_cmd_running_ticks");
  CHECK(lua_tointeger(L, -1) == 2);
  lua_pop(L, 1);
}

TEST_CASE("import_media_file: 動画/音声/画像を内容から自動判別してEntityを追加し、既存プロジェクトを消さない") {
  Project::New();
  Composition* main_comp = Composition::GetActiveComp();
  REQUIRE(main_comp);

  // 事前に追加しておく別Entity。import_media_file()がProject::New()を呼んでしまうとこれが消える
  auto pre_existing = add_new_shape_track("pre_existing", 0, 100, ShapeType_Rect);
  REQUIRE(pre_existing);
  uint64_t pre_existing_guid = pre_existing->guid_;

  auto movie_entt = import_media_file("../assets/movies/big_buck_bunny_360_10s.mp4");
  REQUIRE(movie_entt);
  CHECK(movie_entt->getType() == EntityType_Movie);

  auto audio_entt = import_media_file("../assets/audio/file_example_WAV_1MG.wav");
  REQUIRE(audio_entt);
  CHECK(audio_entt->getType() == EntityType_Audio);

  auto image_entt = import_media_file("../assets/images/blender_png.png");
  REQUIRE(image_entt);
  CHECK(image_entt->getType() == EntityType_Image);

  // pre_existingがProject::New()で消されずまだcompositionに残っていることを確認する
  bool found = false;
  for(const auto& e : main_comp->get_all_entities())
    if(e->guid_ == pre_existing_guid) found = true;
  CHECK(found);
}

TEST_CASE("統合テスト: save_project_cmdをrun_command()で呼び出すと保存先が既にある場合は上書き保存される") {
  fs_create_directory("/tmp/opencode");
  const std::string path = "/tmp/opencode/lua_cmd_save_existing.json";
  std::remove(path.c_str());

  Project::New();
  Project::Get()->path = path;
  REQUIRE_FALSE(fs_exists(path)); // まだ保存はされていない(pathフィールドを設定しただけ)

  lua_State* L = make_test_lua();
  REQUIRE(luaL_dofile(L, "../lancher/runtime/shortcuts.lua") == 0);

  CHECK(run_command("save_project_cmd")); // has_project_path()==trueなのでダイアログを開かずsave_project()される
  CHECK(fs_exists(path));

  std::remove(path.c_str());
}

TEST_CASE("統合テスト: save_project_cmdをrun_command()で呼び出すと保存先未設定ならダイアログ経由で保存される") {
  fs_create_directory("/tmp/opencode");
  const std::string path = "/tmp/opencode/lua_cmd_save_via_dialog.json";
  std::remove(path.c_str());

  Project::New();
  Project::Get()->path.clear(); // Project::New()はpathをクリアしないため、他テストの状態が残らないよう明示的に空にする
  REQUIRE(Project::Get()->path.empty());

  lua_State* L = make_test_lua();
  REQUIRE(luaL_dofile(L, "../lancher/runtime/shortcuts.lua") == 0);
  // ネイティブダイアログは自動テストで開けないため、select_save_file_dialogをスタブに差し替える
  std::string stub = "movutl.select_save_file_dialog = function(title, default_name, exts) return \"" + path + "\" end";
  REQUIRE(luaL_dostring(L, stub.c_str()) == 0);

  CHECK(run_command("save_project_cmd"));
  CHECK(fs_exists(path));
  CHECK(Project::Get()->path == path);

  std::remove(path.c_str());
}

TEST_CASE("統合テスト: save_project_as_cmdをrun_command()で呼び出すと指定パスに保存される") {
  fs_create_directory("/tmp/opencode");
  const std::string path = "/tmp/opencode/lua_cmd_save_as.json";
  std::remove(path.c_str());

  Project::New();

  lua_State* L = make_test_lua();
  REQUIRE(luaL_dofile(L, "../lancher/runtime/shortcuts.lua") == 0);
  std::string stub = "movutl.select_save_file_dialog = function(title, default_name, exts) return \"" + path + "\" end";
  REQUIRE(luaL_dostring(L, stub.c_str()) == 0);

  CHECK(run_command("save_project_as_cmd"));
  CHECK(fs_exists(path));
  CHECK(Project::Get()->path == path);

  std::remove(path.c_str());
}

TEST_CASE("統合テスト: open_project_cmdをrun_command()で呼び出すと保存済みプロジェクトが復元される") {
  fs_create_directory("/tmp/opencode");
  const std::string path = "/tmp/opencode/lua_cmd_open.json";
  std::remove(path.c_str());

  // 復元対象: Entityを1つ持つプロジェクトをあらかじめファイルへ保存しておく(ShapeEnttはProject::entities未登録で保存/復元対象外のためImageを使う)
  Project::New();
  auto marker = import_media_file("../assets/images/blender_png.png");
  REQUIRE(marker);
  marker->name = "open_test_marker";
  Project::Save(path.c_str());
  REQUIRE(fs_exists(path));

  // 別のプロジェクト状態にしてから、open_project_cmd経由で上のファイルを読み直す
  Project::New();
  REQUIRE(Entity::Find("open_test_marker") == nullptr);

  lua_State* L = make_test_lua();
  REQUIRE(luaL_dofile(L, "../lancher/runtime/shortcuts.lua") == 0);
  std::string stub = "movutl.select_file_dialog = function(title, exts) return \"" + path + "\" end";
  REQUIRE(luaL_dostring(L, stub.c_str()) == 0);

  CHECK(run_command("open_project_cmd"));
  CHECK(Project::Get()->path == path);
  CHECK(Entity::Find("open_test_marker") != nullptr); // 保存しておいたEntityが復元されている

  std::remove(path.c_str());
}
