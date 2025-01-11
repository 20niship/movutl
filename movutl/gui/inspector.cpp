#include <imgui.h>
#include <movutl/gui/inspector.hpp>

namespace mu {

void InspectorWindow::Update() {
  ImGui::Begin("Inspector");
  ImGui::Text("Inspector");
  ImGui::End();
}

} // namespace mu
