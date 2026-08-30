#include <cctype>
#include <imgui.h>
#include <movutl/app/export_state.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/time.hpp>
#include <movutl/gui/gui.hpp>
#include <sstream>
#include <unordered_map>

namespace mu::detail {

namespace {

struct KeyChord {
  bool ctrl = false, shift = false, alt = false, super = false;
  ImGuiKey key = ImGuiKey_None;

  bool operator==(const KeyChord& o) const { return ctrl == o.ctrl && shift == o.shift && alt == o.alt && super == o.super && key == o.key; }
};

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

// "ctrl+shift+a" のような1キー分の表記をパースする
KeyChord parse_chord(const std::string& token) {
  KeyChord chord;
  std::vector<std::string> parts;
  std::stringstream ss(token);
  std::string part;
  while(std::getline(ss, part, '+')) parts.push_back(to_lower(part));
  if(parts.empty()) return chord;
  for(size_t i = 0; i + 1 < parts.size(); i++) {
    const auto& m = parts[i];
    if(m == "ctrl" || m == "control")
      chord.ctrl = true;
    else if(m == "shift")
      chord.shift = true;
    else if(m == "alt" || m == "option")
      chord.alt = true;
    else if(m == "cmd" || m == "super" || m == "win")
      chord.super = true;
  }
  chord.key = key_from_name(parts.back());
  return chord;
}

// "g g" や "ctrl+k ctrl+s" のような空白区切りの複数キーシーケンスをパースする
std::vector<KeyChord> parse_shortcut(const std::string& shortcut) {
  std::vector<KeyChord> chords;
  std::stringstream ss(shortcut);
  std::string token;
  while(ss >> token) chords.push_back(parse_chord(token));
  return chords;
}

// 入力中のマルチキーシーケンスの進捗
std::vector<KeyChord> pending_sequence;
double last_key_time                     = 0.0;
constexpr double kSequenceTimeoutSeconds = 1.0;

} // namespace

// ImGui::IsKeyChordPressedには頼らず、押されたキーを自前でトラッキングしてvim風の複数キーシーケンスに対応する
void process_command_shortcuts() {
  if(is_exporting()) {
    // エクスポート中は編集系ショートカットを一切受け付けず、Escキーのみキャンセルとして扱う
    if(ImGui::IsKeyPressed(ImGuiKey_Escape, false)) request_cancel_export();
    return;
  }
  if(ImGui::IsAnyItemActive()) return;     // テキスト入力中などはショートカットを無視する
  if(ImGui::GetIO().WantTextInput) return; // InputText編集中の文字キー衝突(例: "s")を防ぐ

  double now = mu_now_seconds();
  if(!pending_sequence.empty() && now - last_key_time > kSequenceTimeoutSeconds) pending_sequence.clear();

  ImGuiKey pressed_key = ImGuiKey_None;
  for(int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; k++) {
    auto key = (ImGuiKey)k;
    if(key == ImGuiKey_LeftCtrl || key == ImGuiKey_RightCtrl || key == ImGuiKey_LeftShift || key == ImGuiKey_RightShift || key == ImGuiKey_LeftAlt || key == ImGuiKey_RightAlt || key == ImGuiKey_LeftSuper || key == ImGuiKey_RightSuper) continue; // 修飾キー単体は対象外(chordのmodifierとして扱う)
    if(ImGui::IsKeyPressed(key, false)) {
      pressed_key = key;
      break;
    }
  }
  if(pressed_key == ImGuiKey_None) return;

  const auto& io = ImGui::GetIO();
  KeyChord chord{io.KeyCtrl, io.KeyShift, io.KeyAlt, io.KeySuper, pressed_key};
  pending_sequence.push_back(chord);
  last_key_time = now;

  bool any_prefix_match = false;
  for(const auto& info : get_command_infos()) {
    if(info.shortcut.empty()) continue;
    auto chords = parse_shortcut(info.shortcut);
    if(chords.size() < pending_sequence.size()) continue;
    bool is_prefix = true;
    for(size_t i = 0; i < pending_sequence.size(); i++) {
      if(!(chords[i] == pending_sequence[i])) {
        is_prefix = false;
        break;
      }
    }
    if(!is_prefix) continue;
    any_prefix_match = true;
    if(chords.size() == pending_sequence.size()) {
      run_command(info.id.c_str());
      pending_sequence.clear();
      return;
    }
  }
  if(!any_prefix_match) pending_sequence.clear();
}

} // namespace mu::detail
