#pragma once

extern "C" {
struct lua_State;
}

namespace mu::detail {
// movutl.register_window/list_custom_objectsをLuaへ手動バインドする
void bind_lua_ui_panel_api(lua_State* L);
} // namespace mu::detail
