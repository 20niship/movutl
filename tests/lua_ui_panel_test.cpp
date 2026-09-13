#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <doctest/doctest.h>
#include <movutl/binding/binding.hpp>
#include <movutl/gui/gui.hpp>

using namespace mu;

namespace {
// GUIManagerのシングルトンはプロセス全体で共有されるため、close()しない(意図的リーク。lua_command_testと同じ方針)
lua_State* make_test_lua() {
  lua_State* L = luaL_newstate();
  luaL_openlibs(L);
  detail::generated_lua_binding_movutl(L);
  detail::bind_lua_ui_panel_api(L);
  return L;
}
} // namespace

TEST_CASE("movutl.register_window: LuaのdefからUIPanelがGUIManager::panelsへ1件追加される") {
  lua_State* L  = make_test_lua();
  size_t before = GUIManager::Get()->panels.size();

  const char* script = R"(
    movutl.register_window("test_window", { update = function(self) end })
  )";
  REQUIRE(luaL_dostring(L, script) == 0);

  CHECK(GUIManager::Get()->panels.size() == before + 1);
}

TEST_CASE("movutl.list_custom_objects: エラーなく呼び出せ、テーブルを返す") {
  lua_State* L = make_test_lua();

  const char* script = R"(
    local names = movutl.list_custom_objects()
  )";
  int rc             = luaL_dostring(L, script);
  if(rc != 0) printf("LUA ERROR: %s\n", lua_tostring(L, -1));
  CHECK(rc == 0);
}
