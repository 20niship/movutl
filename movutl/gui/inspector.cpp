#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/widgets.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

void InspectorWindow::Update() {
  ImGui::Begin(ICON_FA_PLUG " エフェクト制御");
  auto entts = get_selected_entts();
  if(entts.empty()) {
    ImGui::TextDisabled("オブジェクトが選択されていません");
    ImGui::End();
    return;
  }
  Ref<Entity> e = entts[0];

  {
    const std::string str = get_entt_icon(e) + std::string(" ") + e->name.c_str();
    ImGui::SmallButton(str.c_str());
  }
  wd_entt_props_editor(e.get());

  for(int i = 0; i < e->trk.filters.size(); i++) {
    auto& f = e->trk.filters[i];
    MU_ASSERT(f.plg_ != nullptr);
    ImGui::PushID(i);
    std::string FX_ICON = ICON_FA_PLUG " ";
    std::string str = FX_ICON + f.plg_->name;
    { // Fxアイコンをクリックした時はエフェクトのON/OFFを切り替える
      auto c = ImGui::GetCursorScreenPos();
      auto h = ImGui::GetTextLineHeight();
      ImRect R(c, ImVec2(c.x + 20, c.y + h));
      bool hovers = ImGui::IsMouseHoveringRect(R.Min, R.Max);
      if(hovers) {
        auto dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 20));
        ImGui::SetTooltip("エフェクト %s を有効/無効にします", f.plg_->name);
        if(ImGui::IsMouseClicked(0)) f.enabled = !f.enabled;
      }
    }
    if(ImGui::TreeNode(str.c_str())) {
      int size_ = std::min<int>(f.props.size(), e->trk.filters[i].props.size());
      for(int k = 0; k < size_; k++) {
        auto p = f.props[k];
        auto info = f.plg_->props[k];
        ImGui::PushID(k);
        ImGui::Text("%s", info.name.c_str());
      }
    }
    ImGui::PopID();
  }

  ImGui::End();
}

} // namespace mu
