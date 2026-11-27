#include <cctype>
#include <imgui.h>
#include <movutl/app/export_state.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/time.hpp>
#include <movutl/gui/commands_input.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/workspace_presets.hpp>
#include <sstream>
#include <unordered_map>

namespace mu::detail {

namespace {

std::string to_lower(std::string s) {
  for(auto& c : s) c = (char)std::tolower((unsigned char)c);
  return s;
}

ImGuiKey key_from_name(const std::string& name) {
  static const std::unordered_map<std::string, ImGuiKey> table = {
    {"space", ImGuiKey_Space},
    {"esc", ImGuiKey_Escape},
    {"escape", ImGuiKey_Escape},
    {"enter", ImGuiKey_Enter},
    {"return", ImGuiKey_Enter},
    {"tab", ImGuiKey_Tab},
    {"backspace", ImGuiKey_Backspace},
    {"delete", ImGuiKey_Delete},
    {"del", ImGuiKey_Delete},
    {"left", ImGuiKey_LeftArrow},
    {"right", ImGuiKey_RightArrow},
    {"up", ImGuiKey_UpArrow},
    {"down", ImGuiKey_DownArrow},
    {"home", ImGuiKey_Home},
    {"end", ImGuiKey_End},
    {"pageup", ImGuiKey_PageUp},
    {"pagedown", ImGuiKey_PageDown},
    {"f1", ImGuiKey_F1},
    {"f2", ImGuiKey_F2},
    {"f3", ImGuiKey_F3},
    {"f4", ImGuiKey_F4},
    {"f5", ImGuiKey_F5},
    {"f6", ImGuiKey_F6},
    {"f7", ImGuiKey_F7},
    {"f8", ImGuiKey_F8},
    {"f9", ImGuiKey_F9},
    {"f10", ImGuiKey_F10},
    {"f11", ImGuiKey_F11},
    {"f12", ImGuiKey_F12},
    {"[", ImGuiKey_LeftBracket},
    {"]", ImGuiKey_RightBracket},
    {"/", ImGuiKey_Slash},
    {"\\", ImGuiKey_Backslash},
    {"-", ImGuiKey_Minus},
    {"=", ImGuiKey_Equal},
    {"plus", ImGuiKey_Equal},
    {",", ImGuiKey_Comma},
    {".", ImGuiKey_Period},
    {";", ImGuiKey_Semicolon},
    {"'", ImGuiKey_Apostrophe},
    {"`", ImGuiKey_GraveAccent},
  };
  auto it = table.find(name);
  if(it != table.end()) return it->second;
  if(name.size() == 1) {
    char c = name[0];
    if(c >= 'a' && c <= 'z') return (ImGuiKey)(ImGuiKey_A + (c - 'a'));
    if(c >= '0' && c <= '9') return (ImGuiKey)(ImGuiKey_0 + (c - '0'));
  }
  return ImGuiKey_None;
}

// "ctrl+shift+a" のような1キー分の表記をパースする。"+"自体は "+" "ctrl++" "plus" のいずれでも指定できる
KeyChord parse_chord(std::string token) {
  const size_t n = token.size();
  if(n > 0 && token[n - 1] == '+' && (n == 1 || token[n - 2] == '+')) token.replace(n - 1, 1, "plus");

  KeyChord chord;
  std::vector<std::string> parts;
  std::stringstream ss(token);
  std::string part;
  while(std::getline(ss, part, '+')) parts.push_back(to_lower(part));
  for(size_t i = 0; i < parts.size(); i++) {
    const auto& m = parts[i];
    if(m == "ctrl" || m == "control")
      chord.ctrl = true;
    else if(m == "shift")
      chord.shift = true;
    else if(m == "alt" || m == "option")
      chord.alt = true;
    else if(m == "cmd" || m == "super" || m == "win")
      chord.super = true;
    else if(i + 1 == parts.size()) {
      chord.key = key_from_name(m);
      if(chord.key == ImGuiKey_None) return {};
      chord.any_shift = m == "plus";
    }
  }
  return chord;
}

// 入力中のマルチキーシーケンスの進捗
std::vector<KeyChord> pending_sequence;
double last_key_time                     = 0.0;
constexpr double kSequenceTimeoutSeconds = 1.0;

// set_shortcut_context()はパネル描画中に呼ばれるので、次フレームのショートカット判定で使う
std::string current_context, next_context;

// ペイント系ワークスペースのSpaceは手のひらと兼用なので、短く押して離した時に実行する
std::string tap_command;
double tap_time                     = 0.0;
constexpr double kTapTimeoutSeconds = 0.3;

} // namespace

std::vector<KeyChord> parse_shortcut(const std::string& shortcut) {
  std::vector<KeyChord> chords;
  std::stringstream ss(shortcut);
  std::string token;
  while(ss >> token) chords.push_back(parse_chord(token));
  return chords;
}

ImGuiKey normalize_key(ImGuiKey key) {
  switch(key) {
    case ImGuiKey_KeypadAdd: return ImGuiKey_Equal;
    case ImGuiKey_KeypadSubtract: return ImGuiKey_Minus;
    case ImGuiKey_KeypadDivide: return ImGuiKey_Slash;
    case ImGuiKey_KeypadDecimal: return ImGuiKey_Period;
    default: return key;
  }
}

const CommandInfo* find_shortcut_command(const std::vector<CommandInfo>& infos, const std::vector<KeyChord>& sequence, const std::string& context, bool& any_prefix, const std::string& workspace) {
  any_prefix                      = false;
  const CommandInfo* in_workspace = nullptr;
  const CommandInfo* fallback     = nullptr;
  for(const auto& info : infos) {
    if(info.shortcut.empty()) continue;
    if(!info.context.empty() && info.context != context && info.context != workspace) continue;
    const auto chords = parse_shortcut(info.shortcut);
    if(chords.size() < sequence.size()) continue;
    bool is_prefix = true;
    for(size_t i = 0; i < sequence.size() && is_prefix; i++) is_prefix = chords[i].matches(sequence[i]);
    if(!is_prefix) continue;
    any_prefix = true;
    if(chords.size() != sequence.size()) continue;
    if(!info.context.empty() && info.context == context) return &info;
    auto& slot = info.context.empty() ? fallback : in_workspace;
    if(!slot) slot = &info;
  }
  return in_workspace ? in_workspace : fallback;
}

// ImGui::IsKeyChordPressedには頼らず、押されたキーを自前でトラッキングしてvim風の複数キーシーケンスに対応する
void process_command_shortcuts() {
  current_context = std::move(next_context);
  next_context.clear();

  if(is_exporting()) {
    // エクスポート中は編集系ショートカットを一切受け付けず、Escキーのみキャンセルとして扱う
    if(ImGui::IsKeyPressed(ImGuiKey_Escape, false)) request_cancel_export();
    return;
  }
  double now = mu_now_seconds();
  if(!tap_command.empty()) {
    if(ImGui::IsAnyMouseDown() || now - tap_time > kTapTimeoutSeconds) {
      tap_command.clear(); // ドラッグ(手のひら等)か長押し
    } else if(ImGui::IsKeyReleased(ImGuiKey_Space)) {
      run_command(std::exchange(tap_command, {}).c_str());
    }
  }

  if(ImGui::IsAnyItemActive()) return;     // テキスト入力中などはショートカットを無視する
  if(ImGui::GetIO().WantTextInput) return; // InputText編集中の文字キー衝突(例: "s")を防ぐ

  if(!pending_sequence.empty() && now - last_key_time > kSequenceTimeoutSeconds) pending_sequence.clear();

  ImGuiKey pressed_key = ImGuiKey_None;
  for(int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; k++) {
    auto key = (ImGuiKey)k;
    if(key == ImGuiKey_LeftCtrl || key == ImGuiKey_RightCtrl || key == ImGuiKey_LeftShift || key == ImGuiKey_RightShift || key == ImGuiKey_LeftAlt || key == ImGuiKey_RightAlt || key == ImGuiKey_LeftSuper || key == ImGuiKey_RightSuper) continue; // 修飾キー単体は対象外(chordのmodifierとして扱う)
    if(ImGui::IsKeyPressed(key, false)) {
      pressed_key = normalize_key(key);
      break;
    }
  }
  if(pressed_key == ImGuiKey_None) return;

  const auto& io = ImGui::GetIO();
  KeyChord chord{io.KeyCtrl, io.KeyShift, io.KeyAlt, io.KeySuper, pressed_key};
  pending_sequence.push_back(chord);
  last_key_time = now;

  bool any_prefix  = false;
  const bool paint = is_paint_workspace();
  if(const auto* info = find_shortcut_command(get_command_infos(), pending_sequence, current_context, any_prefix, paint ? "paint" : "video")) {
    const std::string id = info->id; // コマンド内で一覧が変わっても安全なようにコピーする
    pending_sequence.clear();
    if(paint && chord.matches({false, false, false, false, ImGuiKey_Space})) {
      tap_command = id;
      tap_time    = now;
      return;
    }
    run_command(id.c_str());
    return;
  }
  if(!any_prefix) pending_sequence.clear();
}

} // namespace mu::detail

namespace mu {

void set_shortcut_context(const std::string& ctx) { detail::next_context = ctx; }

bool is_shortcut_held(const std::string& shortcut) {
  if(is_exporting() || ImGui::GetIO().WantTextInput) return false;
  const auto chords = detail::parse_shortcut(shortcut);
  if(chords.size() != 1) return false;
  const auto& c  = chords[0];
  const auto& io = ImGui::GetIO();
  if(c.ctrl != io.KeyCtrl || (!c.any_shift && c.shift != io.KeyShift) || c.alt != io.KeyAlt || c.super != io.KeySuper) return false;
  if(c.key == ImGuiKey_None) return c.ctrl || c.shift || c.alt || c.super;
  for(int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; k++)
    if(detail::normalize_key((ImGuiKey)k) == c.key && ImGui::IsKeyDown((ImGuiKey)k)) return true;
  return false;
}

} // namespace mu
