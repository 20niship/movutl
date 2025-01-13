#pragma once
extern "C" {
struct lua_State;
}

namespace mu {
namespace detail {
void init_lua_binding();
void terminate_lua_binding();
void generated_lua_binding_(lua_State* lua);
} // namespace detail
} // namespace mu
