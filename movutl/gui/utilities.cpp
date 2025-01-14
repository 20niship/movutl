#include <imgui.h>
#include <movutl/gui/utilities.hpp>

namespace mu {

void UtilityWindow ::Update() {
  ImGui::Begin("Utilities");
  ImGui::Text("Hello, world!");
  ImGui::End();
}

} // namespace mu
