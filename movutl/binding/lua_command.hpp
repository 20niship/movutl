#pragma once

extern "C" {
struct lua_State;
}

namespace mu::detail {
// movutl.register_command/select_file_dialog/select_save_file_dialogをLuaへ手動バインドする
void bind_lua_command_api(lua_State* L);
} // namespace mu::detail
