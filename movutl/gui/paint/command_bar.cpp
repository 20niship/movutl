#include <IconsFontAwesome6.h>
#include <imgui_internal.h>
#include <movutl/core/command.hpp>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

void CommandBarPanel::Update() {
  // クリスタと同じくタブの無い細い帯にする(配置の左上の印を押すとタブが出て移動できる)
  ImGuiWindowClass bar;
  bar.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_AutoHideTabBar | ImGuiDockNodeFlags_NoWindowMenuButton | ImGuiDockNodeFlags_NoCloseButton;
  ImGui::SetNextWindowClass(&bar);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));
  const bool visible = ImGui::Begin(paint_window_title("command_bar").c_str(), &open);
  ImGui::PopStyleVar();
  if(visible) {
    struct Item {
      const char* id;
      const char* icon;
      const char* label;
      bool (*enabled)();
    };
    static constexpr Item kItems[] = {
      {"paint_new", ICON_FA_FILE, "新規", nullptr}, {"paint_open", ICON_FA_FOLDER_OPEN, "開く", nullptr}, {"paint_save", ICON_FA_FLOPPY_DISK, "保存", nullptr},
      {nullptr, nullptr, nullptr, nullptr},         {"undo", ICON_FA_ROTATE_LEFT, "取り消し", can_undo},  {"redo", ICON_FA_ROTATE_RIGHT, "やり直し", can_redo},
    };
    bool first = true, separate = false;
    for(const auto& item : kItems) {
      if(!item.id) {
        separate = !first;
        continue;
      }
      if(!has_command(item.id)) continue; // 登録されていないコマンドは出さない
      if(!first) ImGui::SameLine(0, separate ? 24.0f : -1.0f);
      first = separate = false;
      ImGui::BeginDisabled(item.enabled && !item.enabled());
      if(ImGui::Button((std::string(item.icon) + "##" + item.id).c_str())) run_command(item.id);
      ImGui::EndDisabled();
      for(const auto& info : get_command_infos()) {
        if(info.id != item.id) continue;
        if(info.shortcut.empty())
          ImGui::SetItemTooltip("%s", item.label);
        else
          ImGui::SetItemTooltip("%s (%s)", item.label, info.shortcut.c_str());
      }
    }
  }
  ImGui::End();
}

} // namespace mu
