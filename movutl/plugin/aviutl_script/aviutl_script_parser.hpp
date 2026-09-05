#pragma once

#include <string>
#include <vector>

namespace mu::detail {

struct AviUtlTrackDef {
  std::string name;
  float min_value     = 0.0f;
  float max_value     = 100.0f;
  float default_value = 0.0f;
  float step          = 1.0f;
};

struct AviUtlCheckDef {
  std::string name;
  bool default_value = false;
};

// AviUtlの`@名前`スクリプトブロック1個分(トラックバー/チェックボックス定義+Lua本体)
struct AviUtlScriptDef {
  std::string name;
  std::vector<AviUtlTrackDef> tracks;
  std::vector<AviUtlCheckDef> checks;
  std::string dialog_code; // `--dialog:`の各項目から抽出した変数初期化コード片(UIウィジェット自体は非対応、実行時にlua_bodyの前に結合される)
  std::string lua_body;
};

// .anmファイルのテキストを解析し、`@名前`ブロックごとに分割する(内容自体は変更しない)
std::vector<AviUtlScriptDef> parse_aviutl_script(const std::string& text);

} // namespace mu::detail
