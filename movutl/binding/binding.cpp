#include <movutl/binding/binding.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/logger.hpp>
#include <LuaIntf/LuaIntf.h>

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
}

void terminate_lua_binding() {
  auto ctx = LuaBindingContext::Get();
  lua_close(ctx->lua);
  LOG_F(INFO, "Terminate Lua binding");
}

LuaBindingContext* LuaBindingContext::singleton_ = nullptr;

} // namespace detail
} // namespace mu
