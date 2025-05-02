#pragma once

#include <imgui.h>
#include <movutl/core/defines.hpp>
#include <movutl/core/ref.hpp>
#include <vector>

struct GLFWwindow;
namespace mu {
class Entity;

class UIPanel {
public:
  virtual void Update() = 0;
  UIPanel() = default;
  ~UIPanel() = default;
};

struct GUIManager {
public:
  MOVUTL_DECLARE_SINGLETON(GUIManager);

  GUIManager() = default;
  ~GUIManager() = default;

  std::vector<Ref<UIPanel>> panels;

  int dockspace_id = 0;

  GLFWwindow* glfw_window = nullptr;
  bool should_close = false;
  void init();
  void terminate();
};

const char* get_entt_icon(const Ref<Entity>& entt);

namespace detail {

void init_gui_panels();
void update_gui_panels();
void gui_new_frame();
void gui_render_to_screen();
} // namespace detail

void register_imgui_style(const char* name, const ImGuiStyle& style);
void apply_imgui_style(const char* name);
void remove_imgui_style(const char* name);

void render_main_menu_bar();

} // namespace mu
