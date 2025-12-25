#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/entity.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/undo.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/widgets.hpp>

namespace mu {

bool wd_color_edit(const char* name, Vec4b* col) {
  float cf[4] = {(*col)[0] / 255.0f, (*col)[1] / 255.0f, (*col)[2] / 255.0f, (*col)[3] / 255.0f};
  bool changed = ImGui::ColorEdit4(name, cf);
  if(changed) *col = Vec4b(cf[0] * 255, cf[1] * 255, cf[2] * 255, cf[3] * 255);
  return changed;
}

void wd_entt_props_editor(Entity* e) {
  MU_ASSERT(e);
  ImGui::PushID(e);
  if(e->propinfo_.empty()) {
    e->propinfo_ = e->getPropsInfo();
  }

  // 戻るアクションを指定するために使用
  int64_t focus_id = ImGui::GetFocusID();
  static int64_t last_focus_id = 0;
  static std::string last_property_name = "";
  static Props::Value last_property_value;
  static Entity* last_entity = nullptr;
  int focus_idx_ = -1;
  Props::Value v_comfirmed, v_before;

  const auto p = e->getProps();
  for(int i = 0; i < e->propinfo_.size(); i++) {

    auto pi = e->propinfo_[i];
    if(!p.contains(pi.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name.c_str(), pi.name.c_str());
      continue;
    }
    ImGui::PushID(i + 1);
    bool changed = false;
    Props newp;
    { // type check
      int tt = p.type(pi.name);
      if(tt != pi.type) {
        printf("getprops %s \n", p.summary().c_str());
        printf("info %s \n", e->propinfo_.summary().c_str());
        LOG_F(WARNING, "Property %s -> %s type mismatch %d != %d", e->name.c_str(), pi.name.c_str(), tt, pi.type);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
        ImGui::TextWrapped("Type mismatch: prop %s->%s %d != %d", e->name.c_str(), pi.name.c_str(), tt, pi.type);
        ImGui::PopStyleColor();
        ImGui::PopID();
        continue;
      }
    }
    const char* name_ = pi.dispname.empty() ? pi.name.c_str() : pi.dispname.c_str();
    switch(pi.type) {
      case PropT_Bool: {
        MU_ASSERT(std::holds_alternative<bool>(p[pi.name]));
        bool b = p.get<bool>(pi.name);
        if(ImGui::Checkbox(name_, &b)) {
          newp.set(pi.name, b);
          changed = true;
        }
        break;
      }
      case PropT_Float: {
        MU_ASSERT(std::holds_alternative<float>(p[pi.name]));
        float f = p.get<float>(pi.name);
        if(ImGui::DragFloat(name_, &f, pi.step, pi.min, pi.max)) {
          newp.set(pi.name, f);
          changed = true;
        }
        break;
      }
      case PropT_Int: {
        MU_ASSERT(std::holds_alternative<int>(p[pi.name]));
        int n = p.get<int>(pi.name);
        if(ImGui::InputInt(name_, &n)) {
          newp.set(pi.name, n);
          changed = true;
        }
        break;
      }
      case PropT_String: {
        MU_ASSERT(std::holds_alternative<std::string>(p[pi.name]));
        std::string s = p.get<std::string>(pi.name);
        char buf[256];
        strcpy(buf, s.c_str());
        if(ImGui::InputText(name_, buf, sizeof(buf))) {
          newp.set(pi.name, std::string(buf));
          changed = true;
        }
        break;
      }
      case PropT_Vec2: {
        MU_ASSERT(std::holds_alternative<Vec2>(p[pi.name]));
        Vec2 v = p.get<Vec2>(pi.name);
        if(ImGui::DragFloat2(name_, (float*)&v[0], pi.step, pi.min, pi.max)) {
          newp.set(pi.name, v);
          changed = true;
        }
        break;
      }
      case PropT_Color: {
        MU_ASSERT(std::holds_alternative<Vec4b>(p[pi.name]));
        Vec4b v = p.get<Vec4b>(pi.name);
        if(wd_color_edit(name_, &v)) {
          newp.set(pi.name, v);
          changed = true;
        }
        break;
      }
      case PropT_Vec3: {
        Vec3 v = p.get<Vec3>(pi.name);
        if(ImGui::DragFloat3(name_, (float*)&v[0], pi.step, pi.min, pi.max)) {
          newp.set(pi.name, v);
          changed = true;
        }
        break;
      }
      case PropT_Vec4: {
        Vec4 v = p.get<Vec4>(pi.name);
        if(ImGui::DragFloat4(name_, (float*)&v[0], pi.step, pi.min, pi.max)) {
          newp.set(pi.name, v);
          changed = true;
        }
        break;
      }
      case PropT_Path: {
        std::string s = p.get<std::string>(pi.name);
        char buf[256];
        strcpy(buf, s.c_str());
        if(ImGui::InputText(name_, buf, sizeof(buf))) {
          newp.set(pi.name, std::string(buf));
          changed = true;
        }
      } break;
      case PropT_Entity: {
        auto e = p.get<Entity*>(pi.name);
        if(e) {
          ImGui::Text("%s", e->name.c_str());
        } else {
          ImGui::Text("None");
        }
      }
    }


    auto item_id_ = ImGui::GetItemID();
    
    // フォーカスが当たった時（編集開始）- 元の値を記録
    if(focus_id != last_focus_id && focus_id == item_id_ && focus_id != 0) {
      last_property_value = p.get_(pi.name);
      last_property_name = pi.name;
      last_entity = e;
      LOG_F(1, "start editing %s", pi.name.c_str());
    }
    
    // フォーカスが外れた時（編集終了）- 値が変更されていたらUndoコマンドを作成
    if(item_id_ == last_focus_id && last_focus_id != 0) {
      if(focus_id != item_id_) {
        v_comfirmed = p.get_(pi.name);
        focus_idx_ = i;
        
        // 値が変更されているかチェック
        bool value_changed = false;
        if(last_entity == e && last_property_name == pi.name) {
          // std::variantの比較 - 型と値の両方が一致しているか確認
          value_changed = (last_property_value.index() != v_comfirmed.index()) ||
                         (last_property_value != v_comfirmed);
        }
        
        // 値が変更されていたらUndoコマンドを作成
        if(value_changed) {
          auto undo_cmd = std::make_unique<EntityPropertyChangeCommand>(
            e, pi.name, last_property_value, v_comfirmed
          );
          // コマンドはすでにUIによって実行済みなのでadd_commandを使用
          GetUndoManager().add_command(std::move(undo_cmd));
          LOG_F(1, "end editing %s - value changed, undo command created", pi.name.c_str());
        } else {
          LOG_F(1, "end editing %s - value unchanged", pi.name.c_str());
        }
      }
    }
    
    if(changed) {
      e->setProps(newp);
      e->propinfo_ = e->getPropsInfo();
    }
    ImGui::PopID();
  }
  ImGui::PopID();
  last_focus_id = focus_id;
}

void wd_movie_inspector(Entity* e) {
  MU_ASSERT(e);
}

void wd_image_inspector(Entity* e) {
  MU_ASSERT(e);
}

} // namespace mu
