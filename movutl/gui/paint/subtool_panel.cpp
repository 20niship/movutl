#include <IconsFontAwesome6.h>
#include <algorithm>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

void SubToolPanel::Update() {
  if(!ImGui::Begin(paint_window_title("subtool").c_str(), &open)) {
    ImGui::End();
    return;
  }
  auto& st               = paint_editor();
  const std::string tool = st.tools.active().id;
  BrushPreset* selected  = st.preset();
  if(!selected) {
    ImGui::TextDisabled("%s: サブツールはありません", st.tools.active().name.c_str());
    ImGui::End();
    return;
  }

  // 一覧の変更はPresetのアドレスを無効にするので、描画の後で行う
  enum class Op { None, Select, Duplicate, Remove, Up, Down };
  Op op = Op::None;
  std::string target;
  const auto request = [&](Op o, const std::string& name) { op = o, target = name; };

  const auto presets = st.brushes.of_tool(tool);
  std::vector<std::string> groups;
  for(const auto* p : presets)
    if(std::find(groups.begin(), groups.end(), p->group) == groups.end()) groups.push_back(p->group);

  // 低いパレットでは一覧を優先してボタン列を省く(右クリックメニューで同じ操作ができる)
  const bool buttons      = ImGui::GetContentRegionAvail().y > ImGui::GetFrameHeightWithSpacing() * 3;
  const float footer      = buttons ? ImGui::GetFrameHeightWithSpacing() : 0.0f;
  const std::string shown = tool + "/" + selected->group;
  const bool sync_tab     = shown != shown_; // ツールや選択が外で変わったら、その選択のタブを開く
  shown_                  = shown;

  ImGui::PushID(tool.c_str());
  if(ImGui::BeginTabBar("##groups")) {
    for(const auto& group : groups) {
      const bool is_open = ImGui::BeginTabItem(group.c_str(), nullptr, sync_tab && group == selected->group ? ImGuiTabItemFlags_SetSelected : 0);
      // クリスタと同じく、タブを押したらそのグループのサブツールに持ち替える
      if(ImGui::IsItemClicked() && group != selected->group)
        for(const auto* p : presets)
          if(p->group == group) {
            request(Op::Select, p->name);
            break;
          }
      if(!is_open) continue;
      if(ImGui::BeginChild("##list", ImVec2(0, buttons ? -footer : 0.0f))) {
        for(const auto* p : presets) {
          if(p->group != group) continue;
          ImGui::PushID(p->name.c_str());
          if(ImGui::Selectable(p->name.c_str(), p == selected, ImGuiSelectableFlags_AllowOverlap)) request(Op::Select, p->name);
          // 複製などで選択が移ったら見える位置へ
          if(p == selected && p->name != scrolled_) {
            if(!ImGui::IsItemVisible()) ImGui::SetScrollHereY();
            scrolled_ = p->name;
          }
          // ドラッグで並べ替え: 掴んだまま行の外へ出たら、その向きに1つ動かす
          if(ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
            const float dy = ImGui::GetMouseDragDelta().y;
            if(dy != 0) {
              request(dy < 0 ? Op::Up : Op::Down, p->name);
              ImGui::ResetMouseDragDelta();
            }
          }
          if(ImGui::BeginPopupContextItem("##menu")) {
            if(ImGui::MenuItem("サブツールの複製")) request(Op::Duplicate, p->name);
            if(ImGui::MenuItem("サブツールの削除", nullptr, false, presets.size() > 1)) request(Op::Remove, p->name);
            ImGui::Separator();
            if(ImGui::MenuItem("上へ移動")) request(Op::Up, p->name);
            if(ImGui::MenuItem("下へ移動")) request(Op::Down, p->name);
            ImGui::EndPopup();
          }
          const std::string size = std::to_string((int)(p->current.size + 0.5f));
          ImGui::SameLine(std::max(0.0f, ImGui::GetContentRegionMax().x - ImGui::CalcTextSize(size.c_str()).x));
          ImGui::TextDisabled("%s", size.c_str());
          ImGui::PopID();
        }
      }
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  if(buttons) {
    if(ImGui::Button(ICON_FA_COPY "##dup")) request(Op::Duplicate, selected->name);
    ImGui::SetItemTooltip("サブツールの複製");
    ImGui::SameLine();
    ImGui::BeginDisabled(presets.size() <= 1);
    if(ImGui::Button(ICON_FA_TRASH_CAN "##del")) request(Op::Remove, selected->name);
    ImGui::SetItemTooltip("サブツールの削除");
    ImGui::EndDisabled();
  }

  // 削除は元に戻せないので確認する
  if(op == Op::Remove) {
    ImGui::OpenPopup("サブツールの削除##confirm");
    st.brushes.select(tool, target); // 対象を選んで見せる
  }
  if(ImGui::BeginPopupModal("サブツールの削除##confirm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    const BrushPreset* p = st.brushes.selected(tool);
    ImGui::Text("サブツール「%s」を削除します。", p->name.c_str());
    if(ImGui::Button("削除")) {
      st.brushes.remove(tool, std::string(p->name));
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if(ImGui::Button("キャンセル") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  ImGui::PopID();

  switch(op) {
    case Op::Select: st.brushes.select(tool, target); break;
    case Op::Duplicate: st.brushes.duplicate(tool, target); break;
    case Op::Up: st.brushes.move(tool, target, -1); break;
    case Op::Down: st.brushes.move(tool, target, +1); break;
    default: break;
  }
  ImGui::End();
}

} // namespace mu
