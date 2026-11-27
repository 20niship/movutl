#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/command/exo/exo_report.hpp>
#include <movutl/core/command.hpp>
#include <movutl/gui/export_window.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/paint/document_ui.hpp>
#include <movutl/gui/workspace_presets.hpp>

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
    // ペイント系ワークスペースではキャンバス、それ以外ではプロジェクトが対象(ショートカットも同じ切り分け)
    const bool paint = detail::is_paint_workspace();
    if(ImGui::MenuItem("新規", "Ctrl+N")) run_command(paint ? "paint_new" : "new_project_cmd");
    if(ImGui::MenuItem("開く", "Ctrl+O")) run_command(paint ? "paint_open" : "open_project_cmd"); // OSのファイルダイアログ(shortcuts.luaのCtrl+Oと同経路)
    if(ImGui::MenuItem("保存", "Ctrl+S")) {
      if(paint)
        run_command("paint_save");
      else if(Project::Get()->path.empty())
        open_path_popup([](const char* p) { save_project_as(p); });
      else
        save_project();
    }
    if(ImGui::MenuItem("名前を付けて保存", "Ctrl+Shift+S")) {
      if(paint)
        run_command("paint_save_as");
      else
        open_path_popup([](const char* p) { save_project_as(p); });
    }
    if(ImGui::MenuItem("EXOを読み込む")) run_command("import_exo");
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
    ImGui::MenuItem("ルーラー原点を中央にする", nullptr, &Config::Get()->viewer_ruler_center_origin);
    ImGui::MenuItem("開発者ウィンドウ", nullptr, &Config::Get()->show_developer_window);
    ImGui::MenuItem("FFT", nullptr, &Config::Get()->show_fft_window);
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
  if(ImGui::BeginMenu("ウィンドウ")) {
    if(ImGui::BeginMenu("ワークスペース")) {
      const auto& workspaces = detail::AppMain::Get()->workspaces;
      const auto item        = [](const std::string& name) {
        if(ImGui::MenuItem(workspace_label(name).c_str(), nullptr, GUIManager::Get()->current_workspace == name)) apply_workspace(name.c_str());
      };
      const std::string presets[] = {"Default", "Illust", "Anime", "Photo"};
      for(const auto& name : presets)
        if(workspaces.contains(name)) item(name);
      for(const auto& [name, _] : workspaces) // Luaで登録されたもの
        if(std::find(std::begin(presets), std::end(presets), name) == std::end(presets)) item(name);
      ImGui::Separator();
      if(ImGui::MenuItem("初期配置に戻す")) reset_workspace();
      ImGui::EndMenu();
    }
    if(detail::is_paint_workspace()) {
      ImGui::Separator();
      for(const auto& panel : GUIManager::Get()->panels) {
        const auto& name = panel->window_name;
        if(name.starts_with("###") && detail::is_window_in_workspace(name)) ImGui::MenuItem(paint_window_title(name.substr(3)).c_str(), nullptr, &panel->open);
      }
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
  draw_exo_import_report_dialog();
  draw_document_dialogs();
}
} // namespace mu
