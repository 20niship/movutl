// mu::init()はGLFWウィンドウ作成を含みheadless環境では使えないため、プラグイン登録とLuaバインディングだけ初期化する
#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
#include <LuaIntf/LuaIntf.h>
extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#include <cstdio>
#include <movutl/app/app_impl.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/binding/imgui_custom_values.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;

int main(int argc, char** argv) {
  if(argc < 2) {
    fprintf(stderr, "usage: %s <script.lua>\n", argv[0]);
    return 1;
  }

  detail::init_logger();
  detail::register_default_plugins();
  detail::register_default_filters();
  detail::init_external_plugins();
  detail::activate_all_plugins();

  lua_State* L = luaL_newstate();
  luaL_openlibs(L);
  detail::generated_lua_binding_movutl(L);
  detail::binding_custom_vectors(L);
  if(luaL_dofile(L, argv[1])) {
    fprintf(stderr, "lua error: %s\n", lua_tostring(L, -1));
    return 1;
  }
  return 0;
}
