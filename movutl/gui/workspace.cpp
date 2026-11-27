#include <imgui.h>
#include <imgui_internal.h>
// --
#include <cstring>
#include <filesystem>
#include <map>
#include <movutl/app/app_impl.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>
#include <movutl/gui/workspace_presets.hpp>
#include <set>

namespace mu {

Workspace& Workspace::add_entry(const char* window_name_, int dir_, float ratio_) { return split_entry(window_name_, "", dir_, ratio_); }

Workspace& Workspace::split_entry(const char* window_name_, const char* parent_, int dir_, float ratio_) {
  WorkspaceEntry entry;
  entry.window_name = window_name_;
  entry.parent      = parent_;
  entry.dir         = dir_;
  entry.ratio       = ratio_;
  entries.push_back(entry);
  return *this;
}

Workspace& Workspace::clear_entries() {
  entries.clear();
  return *this;
}

void register_workspace(const char* name, const Workspace& workspace) {
  auto app                   = detail::AppMain::Get();
  app->workspaces[name]      = workspace;
  app->workspaces[name].name = name;
}

void remove_workspace(const char* name) {
  auto app = detail::AppMain::Get();
  app->workspaces.erase(name);
}

namespace {

bool saved_workspace_loaded = false;

// タブを重ねたノードで選択するウィンドウ。ImGuiは後から出現したウィンドウのタブを選びフォーカスも移すので、
// 数フレーム後に選び直して最後(中央)のウィンドウにフォーカスする
std::vector<std::string> tabs_to_select;
int select_tabs_delay = 0;

void select_tabs() {
  if(select_tabs_delay == 0 || --select_tabs_delay > 0) return;
  for(const auto& name : tabs_to_select) {
    auto window = ImGui::FindWindowByName(name.c_str());
    if(!window || !window->DockNode || !window->DockNode->TabBar) continue;
    window->DockNode->TabBar->NextSelectedTabId = window->TabId;
    ImGui::FocusWindow(window);
  }
}

// DockBuilderを使ってワークスペースを実際のドッキングレイアウトへ変換する
void apply_workspace_impl(const Workspace& workspace) {
  const auto dockspace_id = GUIManager::Get()->dockspace_id;
  if(dockspace_id == 0) return;

  ImGui::DockContextClearNodes(ImGui::GetCurrentContext(), 0, true); // 他のワークスペースのフローティングノードも消す
  ImGuiID root = ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  if(root == 0) {
    LOG_F(ERROR, "apply_workspace: failed to add dock node");
    return;
  }
  ImGui::DockBuilderSetNodeSize(root, ImGui::GetMainViewport()->WorkSize);

  // 最後のentryは残り領域全部、それ以外でdir==Noneのentryは直前entryと同ノードへタブとして重ねる
  // 後の分割でノードIDが変わるので、全て分割してからウィンドウをドッキングする
  ImGuiID remaining = root;
  std::map<std::string, ImGuiID> nodes; // ウィンドウ名 -> ノード
  std::string last;                     // 直前にremainingから分割したウィンドウ
  for(size_t i = 0; i < workspace.entries.size(); i++) {
    const auto& entry = workspace.entries[i];
    bool is_last      = (i + 1 == workspace.entries.size());
    if(entry.ratio < 0.0f) continue;
    if(!entry.parent.empty()) {
      if(!nodes.contains(entry.parent)) {
        LOG_F(ERROR, "apply_workspace: parent %s not found (window: %s)", entry.parent.c_str(), entry.window_name.c_str());
        continue;
      }
      const ImGuiID target = nodes[entry.parent];
      if(entry.dir == ImGuiDir_None) {
        nodes[entry.window_name] = target;
        continue;
      }
      ImGuiID rest        = 0;
      const ImGuiID taken = ImGui::DockBuilderSplitNode(target, static_cast<ImGuiDir>(entry.dir), entry.ratio, nullptr, &rest);
      for(auto& [_, id] : nodes)
        if(id == target) id = rest;
      nodes[entry.window_name] = taken;
      continue;
    }
    if(!is_last && entry.dir == ImGuiDir_None && !last.empty()) {
      nodes[entry.window_name] = nodes[last];
      continue;
    }
    if(is_last) {
      nodes[entry.window_name] = remaining;
      break;
    }
    if(entry.ratio >= 1.0f || entry.ratio == 0.0f || entry.dir == ImGuiDir_None) {
      LOG_F(ERROR, "apply_workspace: invalid entry (window: %s, dir: %d, ratio: %f)", entry.window_name.c_str(), entry.dir, entry.ratio);
      continue;
    }
    nodes[entry.window_name] = ImGui::DockBuilderSplitNode(remaining, static_cast<ImGuiDir>(entry.dir), entry.ratio, nullptr, &remaining);
    last                     = entry.window_name;
  }
  std::set<ImGuiID> has_tab; // 初期配置では各ノードに最初に登録したウィンドウを選択する
  tabs_to_select.clear();
  for(const auto& entry : workspace.entries) {
    if(!nodes.contains(entry.window_name)) continue;
    const ImGuiID node = nodes[entry.window_name];
    ImGui::DockBuilderDockWindow(entry.window_name.c_str(), node);
    if(has_tab.insert(node).second) tabs_to_select.push_back(entry.window_name);
  }
  select_tabs_delay = 3;

  ImGui::DockBuilderFinish(root);
}

// iniから読み込んだ配置の選択タブを、パネル描画後に反映する
void restore_selected_tabs() {
  auto& workspaces    = detail::AppMain::Get()->workspaces;
  const auto& current = GUIManager::Get()->current_workspace;
  if(!workspaces.contains(current)) return;
  tabs_to_select.clear();
  for(const auto& entry : workspaces[current].entries) {
    const ImGuiID id    = ImHashStr(entry.window_name.c_str());
    const auto settings = ImGui::FindWindowSettingsByID(id);
    const auto node     = settings ? ImGui::DockBuilderGetNode(settings->DockId) : nullptr;
    if(node && node->SelectedTabId == ImHashStr("#TAB", 0, id)) tabs_to_select.push_back(entry.window_name);
  }
  select_tabs_delay = 3;
}

// 非表示のワークスペースの配置の保存先。表示中の配置はio.IniFilenameに保存される
std::string workspace_ini_path(const std::string& name) { return "imgui_" + name + ".ini"; }

} // namespace

void apply_workspace(const char* name) {
  auto app = detail::AppMain::Get();
  if(!app->workspaces.contains(name)) {
    LOG_F(ERROR, "apply_workspace: workspace %s not found", name);
    return;
  }
  // iniの読み込みはフレーム外で行う必要があるため、常に遅延適用する
  GUIManager::Get()->pending_workspace = name;
}

void reset_workspace() {
  auto* gui = GUIManager::Get();
  // ×で閉じたウィンドウも開き直す。サブツール詳細は必要な時だけ開くので除く
  for(auto& panel : gui->panels)
    if(!panel->window_name.empty() && panel->window_name != "###brush_detail") panel->open = true;
  gui->rebuild_layout = true;
}

namespace detail {

bool is_window_in_workspace(const std::string& window_name) {
  if(window_name.empty()) return true;
  const auto& current = GUIManager::Get()->current_workspace;
  bool in_any         = false;
  for(const auto& [name, ws] : AppMain::Get()->workspaces) {
    for(const auto& entry : ws.entries) {
      if(entry.window_name != window_name) continue;
      if(name == current) return true;
      in_any = true;
    }
  }
  return !in_any; // どのワークスペースにも属さないウィンドウ(Luaで追加したもの等)は常に表示する
}

void init_workspace_settings() {
  ImGuiSettingsHandler handler;
  handler.TypeName   = "movutl";
  handler.TypeHash   = ImHashStr("movutl");
  handler.ReadOpenFn = [](ImGuiContext*, ImGuiSettingsHandler*, const char* name) -> void* { return std::strcmp(name, "Workspace") == 0 ? GUIManager::Get() : nullptr; };
  handler.ReadLineFn = [](ImGuiContext*, ImGuiSettingsHandler*, void* entry, const char* line) {
    if(std::strncmp(line, "Current=", 8) != 0) return;
    static_cast<GUIManager*>(entry)->current_workspace = line + 8;
    saved_workspace_loaded                             = true;
  };
  handler.WriteAllFn = [](ImGuiContext*, ImGuiSettingsHandler* h, ImGuiTextBuffer* buf) { buf->appendf("[%s][Workspace]\nCurrent=%s\n\n", h->TypeName, GUIManager::Get()->current_workspace.c_str()); };
  ImGui::AddSettingsHandler(&handler);
}

void init_first_frame_layout() {
  auto* gui = GUIManager::Get();
  if(!AppMain::Get()->workspaces.contains(gui->current_workspace)) {
    gui->rebuild_layout = true; // iniの名前が未登録。build_workspace_layout()がDefaultへ戻す
  } else if(saved_workspace_loaded) {
    restore_selected_tabs();
  } else {
    // DockSpaceが自動生成した空の中央ノードしかなければ配置は未保存とみなす
    const auto node = ImGui::DockBuilderGetNode(gui->dockspace_id);
    if(node == nullptr || (node->IsCentralNode() && node->Windows.empty() && node->ChildNodes[0] == nullptr)) gui->rebuild_layout = true;
  }
}

void switch_pending_workspace() {
  select_tabs();
  auto gui = GUIManager::Get();
  // 初フレームではimgui.iniが未読込なので次のフレームまで待つ
  if(gui->pending_workspace.empty() || gui->dockspace_id == 0) return;
  const std::string next = std::move(gui->pending_workspace);
  gui->pending_workspace.clear();
  if(next == gui->current_workspace) return;

  LOG_F(1, "switch workspace: %s", next.c_str());
  const bool persist = ImGui::GetIO().IniFilename != nullptr;
  if(persist) ImGui::SaveIniSettingsToDisk(workspace_ini_path(gui->current_workspace).c_str());
  const std::string path = workspace_ini_path(next);
  select_tabs_delay      = 0;
  const bool saved       = persist && std::filesystem::exists(path);
  if(saved) ImGui::LoadIniSettingsFromDisk(path.c_str());
  gui->current_workspace = next;
  if(saved)
    restore_selected_tabs();
  else
    gui->rebuild_layout = true;
  viewer_cursor().valid = false; // Viewerを描画しないワークスペースで座標表示が残らないようにする
  ImGui::MarkIniSettingsDirty();
}

void build_workspace_layout() {
  auto gui         = GUIManager::Get();
  auto& workspaces = AppMain::Get()->workspaces;
  if(!workspaces.contains(gui->current_workspace)) gui->current_workspace = "Default";
  if(!workspaces.contains(gui->current_workspace)) return;
  apply_workspace_impl(workspaces[gui->current_workspace]);
}

} // namespace detail
} // namespace mu
