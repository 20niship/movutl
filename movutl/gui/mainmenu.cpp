#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/gui/export_window.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {

namespace {
char path_popup_buf[512]             = "";
bool show_path_popup                 = false;
void (*on_path_confirm)(const char*) = nullptr;

void open_path_popup(void (*on_confirm)(const char*)) {
  path_popup_buf[0] = '\0';
  on_path_confirm   = on_confirm;
  show_path_popup   = true;
}
} // namespace

void render_main_menu_bar() {
  if(!ImGui::BeginMainMenuBar()) {
    return;
  }
  ImGui::BeginDisabled(is_exporting()); // エクスポート中はプロジェクト操作を一切禁止する(キャンセルはExportWindow/Escキーで行う)
  if(ImGui::BeginMenu("ファイル")) {
    if(ImGui::MenuItem("新規", "Ctrl+N")) new_project();
    if(ImGui::MenuItem("開く", "Ctrl+O")) open_path_popup([](const char* p) { open_project(p); });
    if(ImGui::MenuItem("保存", "Ctrl+S")) {
      if(Project::Get()->path.empty())
        open_path_popup([](const char* p) { save_project_as(p); });
      else
        save_project();
    }
    if(ImGui::MenuItem("名前を付けて保存", "Ctrl+Shift+S")) open_path_popup([](const char* p) { save_project_as(p); });
    if(ImGui::BeginMenu("エクスポート")) {
      auto& plugins = detail::AppMain::Get()->output_plugins;
      if(plugins.empty()) ImGui::TextDisabled("出力プラグインが登録されていません");
      for(int i = 0; i < (int)plugins.size(); i++)
        if(ImGui::MenuItem(plugins[i].name)) open_export_window(i);
      ImGui::EndMenu();
    }
    if(ImGui::MenuItem("終了", "Ctrl+Q")) GUIManager::Get()->should_close = true;
    ImGui::EndMenu();
  }
  if(ImGui::BeginMenu("表示")) {
    if(ImGui::MenuItem("ツールバー")) {
    }
    if(ImGui::MenuItem("ステータスバー")) {
    }
    ImGui::MenuItem("ルーラー表示", nullptr, &Config::Get()->show_viewer_ruler);
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
  ImGui::EndDisabled();
  ImGui::EndMainMenuBar();

  if(show_path_popup) {
    ImGui::OpenPopup("パス入力");
    show_path_popup = false;
  }
  if(ImGui::BeginPopupModal("パス入力", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::InputText("path", path_popup_buf, sizeof(path_popup_buf));
    if(ImGui::Button("OK") && on_path_confirm) {
      on_path_confirm(path_popup_buf);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if(ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
}
} // namespace mu
