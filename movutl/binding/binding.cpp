#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0
extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}
#include <LuaIntf/LuaIntf.h>
#include <movutl/binding/binding.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/logger.hpp>

namespace mu {
namespace detail {
struct LuaBindingContext {
public:
  MOVUTL_DECLARE_SINGLETON(LuaBindingContext);
  LuaBindingContext() = default;
  ~LuaBindingContext() = default;
  lua_State* lua = nullptr;
};

void init_lua_binding() {
  auto ctx = LuaBindingContext::Get();
  ctx->lua = luaL_newstate();
  luaL_openlibs(ctx->lua);

  // load lua file
  generated_lua_binding_movutl(ctx->lua);
  generated_lua_binding_imgui(ctx->lua);

  const char* init_file = "../lancher/runtime/init.lua";
  if(luaL_dofile(ctx->lua, init_file)) {
    LOG_F(ERROR, "Failed to load lua file: %s", init_file);
    // get and print error
    const char* err = lua_tostring(ctx->lua, -1);
    LOG_F(ERROR, "Error: %s", err);
    return;
  }
}

void terminate_lua_binding() {
  auto ctx = LuaBindingContext::Get();
  lua_close(ctx->lua);
  LOG_F(INFO, "Terminate Lua binding");
}

LuaBindingContext* LuaBindingContext::singleton_ = nullptr;

} // namespace detail
} // namespace mu
