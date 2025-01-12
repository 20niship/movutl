#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {

void TimelineWindow::header() {}

void TimelineWindow::Update() {
  ImGuiWindowClass window_class;
  window_class.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoTabBar;
  ImGui::SetNextWindowClass(&window_class);

  constexpr auto flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
  ImGui::Begin("MOVUTL TIMELINE WINDOW", nullptr, flags);
  auto pj = Project::Get();
  auto cp = Project::GetActiveCompo();
  if(ImGui::BeginTabBar("## MOVUTL TIMELINE TABS")) {
    for(int i = 0; i < pj->compos_.size(); ++i) {
      const std::string str = ICON_FA_FILE + std::string(" ") + pj->compos_[i].name;
      if(ImGui::BeginTabItem(str.c_str())) {
        Project::SetActiveCompo(i);
        cp = Project::GetActiveCompo();
        ImGui::EndTabItem();
      }
    }
    ImGui::EndTabBar();
  }
  MU_ASSERT(cp);

  bool playing = false;
  if(!BeginTimeline(cp->name, &cp->frame, &cp->fstart, &cp->fend, &playing)) {
    EndTimeline();
    ImGui::End();
    return;
  }

  for(int li = 0; li < cp->layers.size(); ++li) {
    auto& layer = cp->layers[li];
    if(!BeginLayer(&layer)) {
      EndLayer();
      continue;
    }
    for(int ei = 0; ei < layer.entts.size(); ++ei) {
      if(!layer.entts[ei]) continue;
      auto& entt = layer.entts[ei];
      bool hovered = BeginTrack(entt);
      if(hovered) {
        ImGui::SetTooltip("Entity %s", entt->name);
      }
      if(hovered && ImGui::IsMouseClicked(0)) {
        select_entt(entt);
      }
      EndTrack();
    }
    EndLayer();
  }
  EndTimeline();
  ImGui::End();
}

} // namespace mu
