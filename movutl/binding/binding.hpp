#pragma once
namespace sol {
class state;
}

namespace mu {
namespace detail {
void init_lua_binding();
void terminate_lua_binding();
void generated_lua_binding_(sol::state* lua);
} // namespace detail
} // namespace mu
