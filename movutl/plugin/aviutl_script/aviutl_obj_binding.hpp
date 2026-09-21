#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/filter.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
  // カスタムオブジェクト専用の「オブジェクトバッファ(fpip->img)+描画先」2面モデル。screenが非nullの時だけ有効で、nullのままなら従来の1面モデル(.anmフィルタ)
  // obj.drawはオブジェクトバッファを壊さず描画先(screenまたはtemp)へ合成し、obj.load("figure"等)/obj.effectはオブジェクトバッファだけを変更する
  Image* screen      = nullptr;     // フレーム全体の出力(obj.setoption("drawtarget","framebuffer"))
  Image* temp        = nullptr;     // 一時バッファ(obj.setoption("drawtarget","tempbuffer",w,h))
  Image* draw_target = nullptr;     // 現在のobj.draw先(screenまたはtemp)
  BlendType blend    = Blend_Alpha; // obj.setoption("blend",...)
  bool screen_drawn  = false;       // obj.draw/drawpolyで描画先へ描いたか(falseのまま終わればオブジェクトバッファを暗黙drawする)
  int rand_counter   = 0;           // seed省略のobj.randが同一フレーム内で呼び出し毎に別の値を返すための連番(フレーム毎に0から数え直すので結果は決定的)
};

// AviUtlObjContext::drawnをtrueにせず、objの現在値(ox/oy/zoom/alpha/rz)でdraw()相当を行う(スクリプト末尾で暗黙的に呼ばれる)
void perform_implicit_draw(lua_State* L, AviUtlObjContext* ctx);

// lua_State上にAviUtl互換の`obj`グローバルテーブルを構築する(track0-3/check0-3は事前にlua_setglobal済みの値をobjにも複製する)
void setup_obj_table(lua_State* L, AviUtlObjContext* ctx);

// AviUtlが提供するobj以外のグローバルヘルパー関数(RGB等)を登録する。lua_State生成直後に1回だけ呼べばよい
void setup_global_functions(lua_State* L);

// 開発者ウィンドウ表示用: 直近に実行したスクリプトのobj変数(実行後の値)を保存/取得する。ワーカースレッドから書かれるため内部でロックする
void store_obj_debug_snapshot(lua_State* L, const std::string& script_name);
std::pair<std::string, std::vector<std::pair<std::string, double>>> load_obj_debug_snapshot();

} // namespace mu::detail
