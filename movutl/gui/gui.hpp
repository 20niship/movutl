#pragma once

#include <movutl/core/defines.hpp>
#include <movutl/core/ref.hpp>
#include <vector>

struct GLFWwindow;
namespace mu {

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

  GLFWwindow* glfw_window = nullptr;
  bool should_close = false;
  void init();
  void terminate();
};

namespace detail {

void init_gui_panels();
void update_gui_panels();
void gui_new_frame();
void gui_render_to_screen();
} // namespace detail

} // namespace mu
