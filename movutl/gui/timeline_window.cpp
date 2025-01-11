#include <imgui.h>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {

void TimelineWindow::header() {
  ImGui::Text("Timeline");
}
void TimelineWindow::Update() {
  ImGui::Begin("Viewer");
  ImGui::Text("Viewer");
  ImGui::End();
}

} // namespace mu
