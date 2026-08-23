#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <cstring>
#include <movutl/asset/entity.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/inspector.hpp>
#include <movutl/gui/widgets.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

namespace {
// movutl/core/string.hpp 削除に伴い、唯一の利用箇所であるここに移動 ([#14])
bool fuzzy_match(const char* src, const char* filter) {
  if(!src || !filter) return true;
  while(*filter) {
    char c = *filter++;
    src    = std::strchr(src, c);
    if(!src) return false;
    src++;
  }
  return true;
}
} // namespace

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
    std::string str     = FX_ICON + f.plg_->name.c_str();
    { // Fxアイコンをクリックした時はエフェクトのON/OFFを切り替える
      auto c = ImGui::GetCursorScreenPos();
      auto h = ImGui::GetTextLineHeight();
      ImRect R(c, ImVec2(c.x + 20, c.y + h));
      bool hovers = ImGui::IsMouseHoveringRect(R.Min, R.Max);
      if(hovers) {
        auto dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(R.Min, R.Max, IM_COL32(255, 255, 255, 20));
        ImGui::SetTooltip("エフェクト %s を有効/無効にします", f.plg_->name.c_str());
        if(ImGui::IsMouseClicked(0)) f.enabled = !f.enabled;
      }
    }
    if(ImGui::TreeNode(str.c_str())) {
      int size_ = std::min<int>(f.props.size(), (int)e->trk.filters[i].plg_->props.fields.size());
      for(int k = 0; k < size_; k++) {
        const auto& info = f.plg_->props.fields[k];
        ImGui::PushID(k);
        { // animation props editor
          const char* label = info.label[0] ? info.label : info.name;
          if(f.props.get_type(k) != info.type) {
            LOG_F(ERROR, "Invalid type: %s", info.name);
          } else if(info.type == cutil::prop_info_of<float>()) {
            float value = f.props.get<float>(k);
            if(ImGui::DragFloat(label, &value, info.drag_speed, info.min_value, info.max_value)) f.props.set_value(k, 0, value);
          } else if(info.type == cutil::prop_info_of<int32_t>()) {
            int value = f.props.get<int>(k);
            if(ImGui::DragInt(label, &value, info.drag_speed)) f.props.set_value(k, 0, value);
          } else if(info.type == cutil::prop_info_of<std::string>()) {
            std::string value = f.props.get<std::string>(k);
            static char buf[256];
            strncpy(buf, value.c_str(), 256);
            if(ImGui::InputText(label, &buf[0], 256)) f.props.set_value(k, 0, std::string(buf));
          } else if(info.type == cutil::prop_info_of<bool>()) {
            bool value = f.props.get<bool>(k);
            if(ImGui::Checkbox(label, &value)) f.props.set_value(k, 0, value);
          } else if(info.type == cutil::prop_info_of<Vec2>()) {
            Vec2 value = f.props.get<Vec2>(k);
            if(ImGui::DragFloat2(label, value.value, info.drag_speed)) f.props.set_value(k, 0, value);
          } else if(info.type == cutil::prop_info_of<Vec3>()) {
            Vec3 value = f.props.get<Vec3>(k);
            if(ImGui::DragFloat3(label, value.value, info.drag_speed)) f.props.set_value(k, 0, value);
          } else if(info.type == cutil::prop_info_of<Vec4>()) {
            Vec4 value = f.props.get<Vec4>(k);
            if(ImGui::DragFloat4(label, value.value, info.drag_speed)) f.props.set_value(k, 0, value);
          }
        }
        ImGui::PopID();
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }

  {
    static bool open_popup        = false;
    static char search_buffer[64] = "";

    // 「Add Filter」ボタン
    if(ImGui::Button("フィルタを追加する")) {
      open_popup = true;
      ImGui::OpenPopup("##INSPECTOR_FILTER_POPUP");
    }
    if(open_popup && ImGui::BeginPopup("##INSPECTOR_FILTER_POPUP")) {
      ImGui::Text("エフェクトを追加する");
      ImGui::Separator();

      ImGui::InputText("Search", search_buffer, IM_ARRAYSIZE(search_buffer));
      auto filters = &detail::AppMain::Get()->filters;
      for(int i = 0; i < filters->size(); i++) {
        const char* name = (*filters)[i].name.c_str();
        if(!fuzzy_match(name, search_buffer)) continue;
        if(ImGui::Selectable(name)) {
          TrackObject::FilterParam fp;
          fp.plg_ = &(*filters)[i];
          fp.props.add_props((*filters)[i].defaults);
          fp.enabled = true;
          e->trk.filters.push_back(fp);
        }
      }
      ImGui::EndPopup();
    }
  }

  ImGui::End();
}

} // namespace mu
