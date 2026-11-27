#pragma once
// ショートカット文字列の解析とコマンド選択。ImGuiの描画やコンテキストに依存しないのでテストできる
#include <imgui.h>
#include <string>
#include <vector>

namespace mu {
struct CommandInfo;
}

namespace mu::detail {

struct KeyChord {
  bool ctrl = false, shift = false, alt = false, super = false;
  ImGuiKey key   = ImGuiKey_None; // Noneで修飾キーのみ("alt"等)
  bool any_shift = false;         // "+"はキー配列でShiftの要否が変わるためShiftを無視する

  // thisを割り当て側として、押されたキーpressedに一致するか
  bool matches(const KeyChord& pressed) const { return ctrl == pressed.ctrl && (any_shift || shift == pressed.shift) && alt == pressed.alt && super == pressed.super && key == pressed.key; }
};

// "g g" や "ctrl+k ctrl+s" のような空白区切りの複数キーシーケンスをパースする。解析できないキーは空のKeyChordになる
std::vector<KeyChord> parse_shortcut(const std::string& shortcut);

// テンキーは対応する通常キーとして扱う
ImGuiKey normalize_key(ImGuiKey key);

// 入力列に完全一致するコマンドを返す。優先順はcontext一致、workspace一致("paint" / "video")、contextが空。他のcontextのコマンドは無視する
// any_prefixには、対象コマンドの中に入力列が前方一致するものがあるかを返す(複数キーの入力待ち判定用)
const CommandInfo* find_shortcut_command(const std::vector<CommandInfo>& infos, const std::vector<KeyChord>& sequence, const std::string& context, bool& any_prefix, const std::string& workspace = "");

} // namespace mu::detail
