#include <movutl/app/app_impl.hpp>
#include <movutl/core/undo.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {
void render_main_menu_bar() {
  if(!ImGui::BeginMainMenuBar()) {
    return;
  }
  if(ImGui::BeginMenu("ファイル")) {
    if(ImGui::MenuItem("新規", "Ctrl+N")) {
    }
    if(ImGui::MenuItem("開く", "Ctrl+O")) {
    }
    if(ImGui::MenuItem("保存", "Ctrl+S")) {
    }
    if(ImGui::MenuItem("名前を付けて保存", "Ctrl+Shift+S")) {
    }
    if(ImGui::MenuItem("終了", "Ctrl+Q")) {
    }
    ImGui::EndMenu();
  }
  if(ImGui::BeginMenu("編集")) {
    // Undo/Redoメニュー項目を追加
    auto& undo_mgr = GetUndoManager();
    bool can_undo = undo_mgr.can_undo();
    bool can_redo = undo_mgr.can_redo();
    
    if(ImGui::MenuItem("元に戻す", "Ctrl+Z", false, can_undo)) {
      undo_mgr.undo();
    }
    if(can_undo && ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", undo_mgr.get_last_undo_description().c_str());
    }
    
    if(ImGui::MenuItem("やり直す", "Ctrl+Y", false, can_redo)) {
      undo_mgr.redo();
    }
    if(can_redo && ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", undo_mgr.get_last_redo_description().c_str());
    }
    
    ImGui::Separator();
    
    if(ImGui::MenuItem("履歴をクリア")) {
      undo_mgr.clear();
    }
    
    ImGui::EndMenu();
  }
  if(ImGui::BeginMenu("表示")) {
    if(ImGui::MenuItem("ツールバー")) {
    }
    if(ImGui::MenuItem("ステータスバー")) {
    }
    if(ImGui::BeginMenu("スタイル")) {
      const auto& styles = detail::AppMain::Get()->imgui_styles;
      for(const auto& style : styles) {
        if(ImGui::MenuItem(style.first.c_str())) {
          apply_imgui_style(style.first.c_str());
        }
      }
      ImGui::EndMenu();
    }
    ImGui::EndMenu();
  }
  ImGui::EndMainMenuBar();
}
} // namespace mu
