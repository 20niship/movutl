#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cstdio>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/command.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/timeline.hpp>
#include <movutl/gui/timeline_window.hpp>
#include <movutl/gui/viewer.hpp>

namespace mu {

namespace {
// タイムライン上部の操作バー: 移動/再生/スナップ/表示幅/追加
void draw_timeline_toolbar(Composition* cp) {
  auto tip = [](const char* t) {
    if(ImGui::IsItemHovered()) ImGui::SetTooltip("%s", t);
  };
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 2));
  if(ImGui::Button(ICON_FA_BACKWARD_FAST)) goto_frame(cp->fstart);
  tip("先頭へ");
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_BACKWARD_STEP)) run_command("frame_step_backward");
  tip("前のフレーム");
  ImGui::SameLine();
  if(ImGui::Button(is_playing() ? ICON_FA_PAUSE : ICON_FA_PLAY)) run_command("play_pause");
  tip(is_playing() ? "一時停止" : "再生");
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_FORWARD_STEP)) run_command("frame_step_forward");
  tip("次のフレーム");
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_FORWARD_FAST)) goto_frame(cp->fend);
  tip("末尾へ");

  ImGui::SameLine();
  ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
  ImGui::SameLine();
  bool* snap = TimelineSnapFlag();
  ImGui::PushStyleColor(ImGuiCol_Button, *snap ? ImVec4(0.20f, 0.45f, 0.85f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Button));
  if(ImGui::Button(ICON_FA_MAGNET)) *snap = !*snap;
  ImGui::PopStyleColor();
  tip("スナップ(プレイヘッド/他クリップの端に吸着。Alt押下中は一時無効)");

  ImGui::SameLine();
  ImGui::TextUnformatted(ICON_FA_MAGNIFYING_GLASS_PLUS);
  ImGui::SameLine();
  FrameT vs = 0, ve = 0;
  if(GetTimelineViewRange(&vs, &ve)) {
    float visible = (float)(ve - vs);
    ImGui::SetNextItemWidth(120);
    if(ImGui::SliderFloat("##tl_zoom", &visible, 10.0f, 20000.0f, "%.0f f", ImGuiSliderFlags_Logarithmic)) SetTimelineVisibleFrames(visible);
    tip("表示幅(フレーム数)。左ほど拡大 / Ctrl+ホイールでも拡大縮小");
  }
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_EXPAND)) RequestTimelineFit();
  tip("全体をフィット");

  ImGui::SameLine();
  ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_PLUS " 追加")) ImGui::OpenPopup("tl_toolbar_add");
  if(ImGui::BeginPopup("tl_toolbar_add")) {
    TimelineAddEntityMenu(cp->frame.load(), -1); // プレイヘッド位置・空きレイヤーへ
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar();
}
} // namespace

void TimelineWindow::header() {}

void TimelineWindow::Update() {
  ImGuiWindowClass window_class;
  window_class.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoTabBar;
  ImGui::SetNextWindowClass(&window_class);

  constexpr auto flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
  ImGui::Begin("MOVUTL TIMELINE WINDOW", nullptr, flags);
  auto pj                    = Project::Get();
  auto cp                    = Project::GetActiveCompo();
  uint32_t close_guid        = 0;
  static int rename_idx      = -1;
  static char rename_buf[64] = {0};
  if(ImGui::BeginTabBar("## MOVUTL TIMELINE TABS", ImGuiTabBarFlags_AutoSelectNewTabs)) {
    for(int i = 0; i < pj->compos_.size(); ++i) {
      const std::string str = ICON_FA_FILE + std::string(" ") + pj->compos_[i]->name.c_str() + "##compo_tab_" + std::to_string(i);
      bool tab_open         = true;
      if(ImGui::BeginTabItem(str.c_str(), pj->compos_.size() > 1 ? &tab_open : nullptr)) { // タブの×で閉じる(最後の1つは閉じられない)
        Project::SetActiveCompo(i);
        cp = Project::GetActiveCompo();
        ImGui::EndTabItem();
      }
      if(!tab_open) close_guid = pj->compos_[i]->guid;
      if(ImGui::BeginPopupContextItem()) {
        if(ImGui::MenuItem(ICON_FA_GEAR " 設定を開く")) pj->compos_[i]->flag = (Composition::Flag)(pj->compos_[i]->flag | Composition::setting_dialog);
        if(ImGui::MenuItem(ICON_FA_PEN " 名前を変更")) {
          rename_idx = i;
          std::snprintf(rename_buf, sizeof(rename_buf), "%s", pj->compos_[i]->name.c_str());
        }
        if(ImGui::MenuItem(ICON_FA_XMARK " 閉じる", nullptr, false, pj->compos_.size() > 1)) close_guid = pj->compos_[i]->guid;
        ImGui::EndPopup();
      }
    }
    if(ImGui::TabItemButton(ICON_FA_PLUS, ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip)) ImGui::OpenPopup("tl_new_comp_menu");
    if(ImGui::BeginPopup("tl_new_comp_menu")) {
      if(ImGui::MenuItem(ICON_FA_FILE_CIRCLE_PLUS " 新規コンポジション")) Project::AddComposition("Composition");
      ImGui::EndPopup();
    }
    ImGui::EndTabBar();
  }
  // タブ名のインライン変更ポップアップ
  if(rename_idx >= 0 && rename_idx < (int)pj->compos_.size()) {
    ImGui::OpenPopup("tl_rename_comp");
    if(ImGui::BeginPopup("tl_rename_comp")) {
      if(ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
      if(ImGui::InputText("##rename", rename_buf, sizeof(rename_buf), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll)) {
        if(rename_buf[0] != '\0') pj->compos_[rename_idx]->name = rename_buf;
        rename_idx = -1;
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    } else {
      rename_idx = -1;
    }
  } else {
    rename_idx = -1;
  }
  if(close_guid != 0) {
    Project::RemoveComposition(close_guid);
    cp = Project::GetActiveCompo();
  }
  MU_ASSERT(cp);

  draw_timeline_toolbar(cp);

  bool playing        = false;
  FrameT frame_before = cp->frame.load();
  FrameT frame_lo     = frame_before;
  if(!BeginTimeline(cp->name.c_str(), &frame_lo, &cp->fstart, &cp->fend, &playing, cp->framerate)) {
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
      auto& entt = layer.entts[ei];
      BeginTrack(entt); // 選択/ドラッグ/ツールチップ/右クリックメニューはBeginTrack内で処理する
      EndTrack();
    }
    EndLayer();
  }

  if(ConsumeTimelineFitRequest()) {
    int mn = cp->fstart, mx = cp->fend;
    bool any = false;
    for(auto& layer : cp->layers) {
      for(auto& e : layer.entts) {
        if(!e) continue;
        if(!any) {
          mn  = e->fstart_;
          mx  = e->fend_;
          any = true;
        } else {
          mn = std::min(mn, e->fstart_);
          mx = std::max(mx, e->fend_);
        }
      }
    }
    SetTimelineViewRange(mn, mx);
  }

  EndTimeline();
  if(frame_lo != frame_before)
    goto_frame(frame_lo); // タイムラインバーのドラッグ等による明示的なシーク。音声もここで追従させる
  else
    cp->frame.store(frame_lo);
  ImGui::End();
}

} // namespace mu
