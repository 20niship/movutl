#include <IconsFontAwesome6.h>
#include <algorithm>
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
      const std::string str = ICON_FA_FILE + std::string(" ") + pj->compos_[i]->name.c_str() + "##compo_tab_" + std::to_string(i);
      if(ImGui::BeginTabItem(str.c_str())) {
        Project::SetActiveCompo(i);
        cp = Project::GetActiveCompo();
        ImGui::EndTabItem();
      }
      if(ImGui::BeginPopupContextItem()) {
        if(ImGui::MenuItem("設定を開く")) pj->compos_[i]->flag = (Composition::Flag)(pj->compos_[i]->flag | Composition::setting_dialog);
        ImGui::EndPopup();
      }
    }
    if(ImGui::TabItemButton(ICON_FA_PLUS, ImGuiTabItemFlags_Trailing)) {
      Project::AddComposition("Composition");
    }
    ImGui::EndTabBar();
  }
  MU_ASSERT(cp);

  if(ImGui::Button("全体表示")) {
    int mn = cp->fstart, mx = cp->fend;
    bool any = false;
    for(auto& layer : cp->layers) {
      for(auto& e : layer.entts) {
        if(!e) continue;
        if(!any) {
          mn  = e->fstart;
          mx  = e->fend;
          any = true;
        } else {
          mn = std::min(mn, e->fstart);
          mx = std::max(mx, e->fend);
        }
      }
    }
    SetTimelineViewRange(mn, mx);
  }

  bool playing        = false;
  FrameT frame_before = cp->frame.load();
  FrameT frame_lo     = frame_before;
  if(!BeginTimeline(cp->name.c_str(), &frame_lo, &cp->fstart, &cp->fend, &playing)) {
    if(frame_lo != frame_before)
      goto_frame(frame_lo); // タイムラインバーのドラッグ等による明示的なシーク。音声もここで追従させる
    else
      cp->frame.store(frame_lo);
    EndTimeline();
    ImGui::End();
    return;
  }

  for(int li = 0; li < cp->layers.size(); ++li) {
    auto& layer = cp->layers[li];
    if(!BeginLayer(cp, li)) {
      EndLayer();
      continue;
    }
    for(int ei = 0; ei < layer.entts.size(); ++ei) {
      if(!layer.entts[ei]) continue;
      auto& entt   = layer.entts[ei];
      bool hovered = BeginTrack(entt);
      if(hovered) {
        ImGui::SetTooltip("Entity %s", entt->name.c_str());
      }
      if(hovered && ImGui::IsMouseClicked(0)) {
        clear_selected_entts();
        select_entt(entt);
      }
      EndTrack();
    }
    EndLayer();
  }
  EndTimeline();
  if(frame_lo != frame_before)
    goto_frame(frame_lo); // タイムラインバーのドラッグ等による明示的なシーク。音声もここで追従させる
  else
    cp->frame.store(frame_lo);
  ImGui::End();
}

} // namespace mu
