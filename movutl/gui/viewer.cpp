#include <movutl/gui/gui.hpp>
#include <movutl/gui/viewer.hpp>
#include <imgui.h>

namespace mu {

void ViewerWindow::Update() {
  ImGui::Begin("Viewer");
  ImGui::Text("Viewer");
  ImGui::End();
}
} // namespace mu
