#pragma once
extern "C" {
struct lua_State;
}

namespace mu {
namespace detail {
void init_lua_binding();
void terminate_lua_binding();
void generated_lua_binding_movutl(lua_State* lua);
void generated_lua_binding_imgui(lua_State* lua);
void lua_binding_imgui_utils(lua_State* lua);
} // namespace detail
} // namespace mu
