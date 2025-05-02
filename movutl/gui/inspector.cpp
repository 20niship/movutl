#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/string.hpp>
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
    std::string str = FX_ICON + f.plg_->name.c_str();
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
      int size_ = std::min<int>(f.props.size(), e->trk.filters[i].props.size());
      for(int k = 0; k < size_; k++) {
        auto p = f.props[k];
        auto info = f.plg_->props[k];
        ImGui::PushID(k);
        { // animation props editor
          switch(info.type) {
            case PropT_Float: {
              if(f.props.get_type(k) != PropT_Float) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Float, f.props.get_type(k));
                break;
              }
              float value = f.props.get<float>(k);
              if(ImGui::DragFloat(info.dispname.c_str(), &value, info.step, info.min, info.max)) f.props.set_value(k, 0, value);
              break;
            }
            case PropT_Int: {
              if(f.props.get_type(k) != PropT_Int) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Int, f.props.get_type(k));
                break;
              }
              int value = f.props.get<int>(k);
              if(ImGui::DragInt(info.dispname.c_str(), &value, info.step)) f.props.set_value(k, 0, value);
              break;
            }
            case PropT_String: {
              if(f.props.get_type(k) != PropT_String) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_String, f.props.get_type(k));
                break;
              }
              std::string value = f.props.get<std::string>(k);
              static char buf[256];
              strncpy(buf, value.c_str(), 256);
              if(ImGui::InputText(info.dispname.c_str(), &buf[0], 256)) f.props.set_value(k, 0, std::string(buf));
              break;
            }

            case PropT_Bool: {
              if(f.props.get_type(k) != PropT_Bool) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Bool, f.props.get_type(k));
                break;
              }
              bool value = f.props.get<bool>(k);
              if(ImGui::Checkbox(info.dispname.c_str(), &value)) f.props.set_value(k, 0, value);
              break;
            }
            case PropT_Vec2: {
              if(f.props.get_type(k) != PropT_Vec2) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Vec2, f.props.get_type(k));
                break;
              }
              Vec2 value = f.props.get<Vec2>(k);
              if(ImGui::DragFloat2(info.dispname.c_str(), &value[0], info.step)) f.props.set_value(k, 0, value);
              break;
            }
            case PropT_Vec3: {
              if(f.props.get_type(k) != PropT_Vec3) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Vec3, f.props.get_type(k));
                break;
              }
              Vec3 value = f.props.get<Vec3>(k);
              if(ImGui::DragFloat3(info.dispname.c_str(), &value[0], info.step)) f.props.set_value(k, 0, value);
              break;
            }
            case PropT_Vec4: {
              if(f.props.get_type(k) != PropT_Vec4) {
                LOG_F(ERROR, "Invalid type: %s -> expected %d, got %d", info.name.c_str(), PropT_Vec4, f.props.get_type(k));
                break;
              }
              Vec4 value = f.props.get<Vec4>(k);
              if(ImGui::DragFloat4(info.dispname.c_str(), &value[0], info.step)) f.props.set_value(k, 0, value);
              break;
            }
          }
        }
        ImGui::PopID();
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
  }

  {
    static bool open_popup = false;
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
          auto pinfo = (*filters)[i].props.get_default();
          fp.props.add_props(pinfo);
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
