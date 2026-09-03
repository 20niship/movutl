#pragma once

#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/filter.hpp>

extern "C" {
struct lua_State;
}

namespace mu::detail {

// obj.*関数から参照する実行コンテキスト(fn_proc呼び出し中のみ有効なスタック上のポインタを渡す)
struct AviUtlObjContext {
  FilterInData* fpip         = nullptr;
  int frame                  = 0;
  const AviUtlScriptDef* def = nullptr;
};

// lua_State上にAviUtl互換の`obj`グローバルテーブルを構築する(track0-3/check0-3は事前にlua_setglobal済みの値をobjにも複製する)
void setup_obj_table(lua_State* L, AviUtlObjContext* ctx);

} // namespace mu::detail
