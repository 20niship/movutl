#pragma once

#include <cutil/ref.hpp>
#include <imgui.h>
#include <movutl/core/defines.hpp>
#include <string>
#include <vector>

struct GLFWwindow;
namespace mu {

using cutil::Ref;
class Entity;

class UIPanel {
public:
  virtual void Update() = 0;
  UIPanel()             = default;
  ~UIPanel()            = default;
};

// ワークスペース内の1ウィンドウ分のドッキング設定
// entriesは登録順に適用され、残り領域からdir方向へratioの割合で領域を確保し、そこへwindow_nameをドッキングする
struct WorkspaceEntry {
  std::string window_name;     // ドッキングするImGuiウィンドウ名
  int dir     = ImGuiDir_None; // 分割方向 (ImGuiDir_Left / Right / Up / Down)
  float ratio = 0.5f;          // 残り領域から確保する割合 (0.0 - 1.0)
};

struct Workspace {
  std::string name; // ワークスペース名
  std::vector<WorkspaceEntry> entries;

  Workspace& add_entry(const char* window_name, int dir, float ratio);
  Workspace& clear_entries();
};

struct GUIManager {
public:
  MOVUTL_DECLARE_SINGLETON(GUIManager);

  GUIManager()  = default;
  ~GUIManager() = default;

  std::vector<Ref<UIPanel>> panels;

  int dockspace_id = 0;

  // 遅延適用用。apply_workspace()が初フレーム前に呼ばれた場合、ここに名前を保持し最初のフレームで適用する
  std::string pending_workspace;

  GLFWwindow* glfw_window = nullptr;
  bool should_close       = false;
  void init();
  void terminate();
};

const char* get_entt_icon(const Ref<Entity>& entt);

namespace detail {

void init_gui_panels();
void update_gui_panels();
void gui_new_frame();
void gui_render_to_screen();
void process_command_shortcuts();
} // namespace detail

void register_imgui_style(const char* name, const ImGuiStyle& style);
void apply_imgui_style(const char* name);
void remove_imgui_style(const char* name);

// ワークスペース(ドッキングレイアウト)の登録・削除・適用
void register_workspace(const char* name, const Workspace& workspace);
void remove_workspace(const char* name);
void apply_workspace(const char* name);

namespace detail {
inline Workspace new_workspace() { return Workspace(); }
inline Workspace workspace_add_entry(Workspace workspace, const char* window_name, int dir, float ratio) {
  workspace.add_entry(window_name, dir, ratio);
  return workspace;
}
} // namespace detail

void render_main_menu_bar();
void render_status_bar();

} // namespace mu
