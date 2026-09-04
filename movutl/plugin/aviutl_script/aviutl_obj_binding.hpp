#pragma once

#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/filter.hpp>
#include <string>
#include <unordered_map>

extern "C" {
struct lua_State;
}

namespace mu::detail {

// obj.*関数から参照する実行コンテキスト(fn_proc呼び出し中のみ有効なスタック上のポインタを渡す)
struct AviUtlObjContext {
  FilterInData* fpip                              = nullptr;
  int frame                                       = 0;
  const AviUtlScriptDef* def                      = nullptr;
  bool drawn                                      = false;   // draw/drawpoly/putpixeldata/copybuffer(obj復元)のいずれかが呼ばれたか。falseのままフレーム処理が終わるとAviUtl本体同様に暗黙でdraw()相当を行う
  std::unordered_map<std::string, Image>* buffers = nullptr; // obj.copybufferの退避先("tmp"/"cache:xxx")。フィルタインスタンス単位でフレームをまたいで保持する
};

// AviUtlObjContext::drawnをtrueにせず、objの現在値(ox/oy/zoom/alpha/rz)でdraw()相当を行う(スクリプト末尾で暗黙的に呼ばれる)
void perform_implicit_draw(lua_State* L, AviUtlObjContext* ctx);

// lua_State上にAviUtl互換の`obj`グローバルテーブルを構築する(track0-3/check0-3は事前にlua_setglobal済みの値をobjにも複製する)
void setup_obj_table(lua_State* L, AviUtlObjContext* ctx);

} // namespace mu::detail
