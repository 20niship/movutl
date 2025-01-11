#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// --
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu::detail {

void init_gui_panels() {
  auto g = GUIManager::Get();
  g->panels = {
    std::make_shared<InspectorWindow>(),
    std::make_shared<TimelineWindow>(),
    std::make_shared<ViewerWindow>(),
  };
}

void update_gui_panels() {
  auto a = GUIManager::Get();
  for(auto& panel : a->panels) {
    panel->Update();
  }

  // 1. Show a simple window
  // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"
  {
    static float f = 0.0f;
    ImGui::Text("Hello, world!");
    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  }
}

} // namespace mu::detail
