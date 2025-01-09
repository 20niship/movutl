#pragma once

#include <stdio.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <movutl/core/defines.hpp>

struct GLFWwindow;

namespace mu {

struct GUIManager {
public:
  MOVUTL_DECLARE_SINGLETON(GUIManager);

  GUIManager() = default;
  ~GUIManager() = default;

  GLFWwindow* glfw_window = nullptr;
  bool should_close = false;
  void init();
  void terminate();
};

} // namespace mu
