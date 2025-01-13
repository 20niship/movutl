#include <movutl/binding/binding.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/logger.hpp>
#include <sol/sol.hpp>

namespace mu {
namespace detail {
struct LuaBindingContext {
public:
  MOVUTL_DECLARE_SINGLETON(LuaBindingContext);
  LuaBindingContext() = default;
  ~LuaBindingContext() = default;
  sol::state lua;
};

void init_lua_binding() {
  auto ctx = LuaBindingContext::Get();
  generated_lua_binding_(&ctx->lua);
}

void terminate_lua_binding() {
  LOG_F(INFO, "Terminate Lua binding");
}

LuaBindingContext* LuaBindingContext::singleton_ = nullptr;

} // namespace detail
} // namespace mu
