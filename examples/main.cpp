// mu::init()はGLFWウィンドウ作成を含みheadless環境では使えないため、プラグイン登録とLuaバインディングだけ初期化する
#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
#include <LuaIntf/LuaIntf.h>
extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#include <cstdio>
#include <cstring>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/binding/imgui_custom_values.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;

int main(int argc, char** argv) {
  if(argc < 2) {
    fprintf(stderr, "usage: %s [--renderer=cpu|vulkan] <script.lua>\n", argv[0]);
    return 1;
  }

  detail::init_logger();
  detail::register_default_plugins();
  detail::register_default_filters();
  detail::init_external_plugins();
  detail::register_aviutl_scripts();
  detail::activate_all_plugins();
  Config::Load(); // movutl_cnf.jsonのrendererをheadless実行(Composition::render_current_frame_main_thread経由)にも反映する
  const char* script = argv[1];
  for(int i = 1; i < argc; i++)
    if(std::strncmp(argv[i], "--renderer=", 11) == 0)
      Config::Get()->renderer = argv[i] + 11;
    else
      script = argv[i];

  lua_State* L = luaL_newstate();
  luaL_openlibs(L);
  detail::generated_lua_binding_movutl(L);
  detail::binding_custom_vectors(L);
  if(luaL_dofile(L, script)) {
    fprintf(stderr, "lua error: %s\n", lua_tostring(L, -1));
    return 1;
  }
  return 0;
}
