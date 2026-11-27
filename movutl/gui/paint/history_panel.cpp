#include <movutl/core/command.hpp>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

void HistoryPanel::Update() {
  if(ImGui::Begin(paint_window_title("history").c_str(), &open)) {
    // ponytail: 毎フレーム名前を複製する。履歴が数万件になって重ければ、変更番号を持たせて作り直しを減らす
    const CommandHistory h = get_command_history();
    const bool changed     = h.applied != last_applied_ || h.names.size() != last_size_;
    last_applied_          = h.applied;
    last_size_             = h.names.size();
    int jump               = -1;
    ImGuiListClipper clipper;
    clipper.Begin((int)h.names.size() + 1); // 先頭は何もしていない状態
    if(changed) clipper.IncludeItemByIndex(h.applied);
    while(clipper.Step()) {
      for(int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++) {
        ImGui::PushID(i);
        const bool undone = i > h.applied;
        if(undone) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        if(ImGui::Selectable(i == 0 ? "(開始時)" : h.names[i - 1].c_str(), i == h.applied)) jump = i;
        if(undone) ImGui::PopStyleColor();
        if(changed && i == h.applied && !ImGui::IsItemVisible()) ImGui::SetScrollHereY();
        ImGui::PopID();
      }
    }
    if(jump >= 0) jump_command_history(jump);
  }
  ImGui::End();
}

} // namespace mu
