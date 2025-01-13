#include <LuaIntf/LuaIntf.h>
#include <movutl/core/string.hpp>

namespace LuaIntf {

template <> struct LuaTypeMapping<CStr> {
  static void push(lua_State* L, const CStr& str) {
    if(str.empty()) {
      lua_pushliteral(L, "");
    } else {
      lua_pushlstring(L, str.c_str(), str.size());
    }
  }

  static CStr get(lua_State* L, int index) {
    size_t len;
    const char* p = luaL_checklstring(L, index, &len);
    return CStr(p, len);
  }
  static CStr opt(lua_State* L, int index, const CStr& def) { return lua_isnoneornil(L, index) ? def : get(L, index); }
};

} // namespace LuaIntf
