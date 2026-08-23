#include <imgui.h>
#include <movutl/core/command.hpp>
#include <movutl/gui/gui.hpp>

namespace mu::detail {

void process_command_shortcuts() {
  if(ImGui::IsAnyItemActive()) return; // テキスト入力中などはショートカットを無視する
  for(const auto& cmd : CommandManager::Get()->commands_) {
    if(cmd->shortcut == ImGuiKey_None) continue;
    if(ImGui::IsKeyChordPressed(cmd->shortcut)) run_command(cmd->id.c_str());
  }
}

} // namespace mu::detail
