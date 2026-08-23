#include <imgui.h>
#include <imgui_internal.h>
// --
#include <movutl/app/app_impl.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {

Workspace& Workspace::add_entry(const char* window_name_, int dir_, float ratio_) {
  WorkspaceEntry entry;
  entry.window_name = window_name_;
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

// DockBuilderを使ってワークスペースを実際のドッキングレイアウトへ変換する
void apply_workspace_impl(const Workspace& workspace) {
  const auto dockspace_id = GUIManager::Get()->dockspace_id;
  if(dockspace_id == 0) return;

  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGuiID root = ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
  if(root == 0) {
    LOG_F(ERROR, "apply_workspace: failed to add dock node");
    return;
  }
  ImGui::DockBuilderSetNodeSize(root, ImGui::GetMainViewport()->WorkSize);

  // entriesを順番に適用し、残り領域からdir方向へratio分ずつ確保していく
  // 最後のentryは残り領域すべてにドッキングされる
  ImGuiID remaining = root;
  for(size_t i = 0; i < workspace.entries.size(); i++) {
    const auto& entry = workspace.entries[i];
    if(i + 1 == workspace.entries.size()) {
      ImGui::DockBuilderDockWindow(entry.window_name.c_str(), remaining);
      break;
    }
    if(entry.ratio >= 1.0f || entry.ratio <= 0.0f || entry.dir == ImGuiDir_None) {
      LOG_F(ERROR, "apply_workspace: invalid entry (window: %s, dir: %d, ratio: %f)", entry.window_name.c_str(), entry.dir, entry.ratio);
      continue;
    }
    ImGuiID taken = ImGui::DockBuilderSplitNode(remaining, static_cast<ImGuiDir>(entry.dir), entry.ratio, nullptr, &remaining);
    ImGui::DockBuilderDockWindow(entry.window_name.c_str(), taken);
  }

  ImGui::DockBuilderFinish(root);
}

} // namespace

void apply_workspace(const char* name) {
  auto app = detail::AppMain::Get();
  if(!app->workspaces.contains(name)) {
    LOG_F(ERROR, "apply_workspace: workspace %s not found", name);
    return;
  }

  // まだフレームが始まっていない場合は遅延適用
  auto gui = GUIManager::Get();
  if(gui->dockspace_id == 0) {
    gui->pending_workspace = name;
    return;
  }

  LOG_F(1, "apply_workspace: %s", name);
  apply_workspace_impl(app->workspaces[name]);
}

} // namespace mu
