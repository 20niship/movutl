#include <IconsFontAwesome6.h>
#include <imgui.h>
#include <movutl/asset/entity.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/logger.hpp>
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
  const auto p = e->getProps();
  for(int i = 0; i < e->propinfo_.size(); i++) {
    auto pi = e->propinfo_[i];
    if(!p.contains(pi.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name, pi.name.c_str());
      continue;
    }
    ImGui::PushID(i + 1);
    bool changed = false;
    Props newp;
    { // type check
      int tt = p.type(pi.name);
      if(tt != pi.type) {
        LOG_F(WARNING, "Property %s -> %s type mismatch %d != %d", e->name, pi.name.c_str(), tt, pi.type);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
        ImGui::TextWrapped("Type mismatch: prop %s->%s %d != %d", e->name, pi.name.c_str(), tt, pi.type);
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
    }

    if(changed) {
      e->setProps(newp);
      e->propinfo_ = e->getPropsInfo();
    }
    ImGui::PopID();
  }
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) {
  MU_ASSERT(e);
}

void wd_image_inspector(Entity* e) {
  MU_ASSERT(e);
}

} // namespace mu
