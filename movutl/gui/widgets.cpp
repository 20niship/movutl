#include <IconsFontAwesome6.h>
#include <algorithm>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/core/assert.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/widgets.hpp>

namespace mu {

bool wd_color_edit(const char* name, Vec4b* col) {
  float cf[4]  = {(*col)[0] / 255.0f, (*col)[1] / 255.0f, (*col)[2] / 255.0f, (*col)[3] / 255.0f};
  bool changed = ImGui::ColorEdit4(name, cf);
  if(changed) *col = Vec4b(cf[0] * 255, cf[1] * 255, cf[2] * 255, cf[3] * 255);
  return changed;
}

void wd_entt_props_editor(Entity* e) {
  MU_ASSERT(e);
  ImGui::PushID(e);

  const cutil::PropInfo* info = e->getPropsInfo();
  if(!info) {
    ImGui::PopID();
    return;
  }

  const auto p = e->getProps();
  for(const auto& f : info->fields) {
    if(!p.contains(f.name)) {
      LOG_F(WARNING, "Property %s -> %s not found", e->name.c_str(), f.name);
      continue;
    }
    ImGui::PushID(f.name);
    bool changed = false;
    cutil::Prop newp;
    const char* name_        = f.label[0] ? f.label : f.name;
    const bool is_path_field = std::string(f.name) == "path" || std::string(f.name) == "path_";

    if(f.type == cutil::prop_info_of<bool>()) {
      bool v = p.get<bool>(f.name);
      if(ImGui::Checkbox(name_, &v)) {
        newp.set<bool>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<float>()) {
      float v = p.get<float>(f.name);
      if(ImGui::DragFloat(name_, &v, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<float>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<int32_t>()) {
      int32_t v = p.get<int32_t>(f.name);
      if(std::string(f.name) == "shape_type_") {
        static const char* kShapeNames[] = {"三角形", "四角形", "六角形", "円", "カスタムパス"};
        int idx                          = std::clamp(v, 0, 4);
        if(ImGui::Combo(name_, &idx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
          newp.set<int32_t>(f.name, idx);
          changed = true;
        }
      } else if(ImGui::InputInt(name_, &v)) {
        newp.set<int32_t>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<uint8_t>()) {
      int v = p.get<uint8_t>(f.name);
      if(ImGui::InputInt(name_, &v)) {
        newp.set<uint8_t>(f.name, static_cast<uint8_t>(std::clamp(v, 0, 255)));
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<std::string>()) {
      std::string s = p.get<std::string>(f.name);
      char buf[256];
      strncpy(buf, s.c_str(), sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';
      if(is_path_field) {
        ImGui::SetNextItemWidth(-40);
        if(ImGui::InputText(name_, buf, sizeof(buf))) {
          newp.set<std::string>(f.name, std::string(buf));
          changed = true;
        }
        ImGui::SameLine();
        if(ImGui::Button(ICON_FA_FOLDER_OPEN "##"
                                             "path_dialog")) {
          std::string picked = select_file_dialog("ファイルを選択", {});
          if(!picked.empty()) {
            newp.set<std::string>(f.name, picked);
            changed = true;
          }
        }
      } else if(ImGui::InputText(name_, buf, sizeof(buf))) {
        newp.set<std::string>(f.name, std::string(buf));
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec2>()) {
      Vec2 v = p.get<Vec2>(f.name);
      if(ImGui::DragFloat2(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec2>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec3>()) {
      Vec3 v = p.get<Vec3>(f.name);
      if(ImGui::DragFloat3(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec3>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4>()) {
      Vec4 v = p.get<Vec4>(f.name);
      if(ImGui::DragFloat4(name_, v.value, f.drag_speed, f.min_value, f.max_value)) {
        newp.set<Vec4>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<Vec4b>()) {
      Vec4b v = p.get<Vec4b>(f.name);
      if(wd_color_edit(name_, &v)) {
        newp.set<Vec4b>(f.name, v);
        changed = true;
      }
    }

    if(changed) {
      {
        std::lock_guard<std::mutex> lock(e->mtx);
        e->setProps(newp);
        if(is_path_field) e->reload_asset(); // パス変更時は新しいファイルを読み込み直す
      }
      if(auto* comp = e->get_comp()) comp->cache.invalidate_all();
    }
    ImGui::PopID();
  }
  ImGui::PopID();
}

void wd_movie_inspector(Entity* e) { MU_ASSERT(e); }

void wd_image_inspector(Entity* e) { MU_ASSERT(e); }

} // namespace mu
