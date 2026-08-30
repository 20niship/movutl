#include <cstring>
#include <imgui.h>
#include <movutl/app/prop_editor.hpp>

namespace mu {

bool draw_props_editor(const cutil::PropInfo* info, cutil::Prop* props) {
  if(info == nullptr) return false;
  bool changed = false;
  for(const auto& f : info->fields) {
    if(!props->contains(f.name)) continue;
    const char* name_ = f.label[0] ? f.label : f.name;
    if(f.type == cutil::prop_info_of<bool>()) {
      bool v = props->get<bool>(f.name);
      if(ImGui::Checkbox(name_, &v)) {
        props->set<bool>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<int32_t>()) {
      int32_t v = props->get<int32_t>(f.name);
      if(ImGui::DragInt(f.name, &v, 1.0f, (int)f.min_value, (int)f.max_value)) {
        props->set<int32_t>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<float>()) {
      float v = props->get<float>(f.name);
      if(ImGui::DragFloat(f.name, &v, f.drag_speed, f.min_value, f.max_value)) {
        props->set<float>(f.name, v);
        changed = true;
      }
    } else if(f.type == cutil::prop_info_of<std::string>()) {
      std::string s = props->get<std::string>(f.name);
      char buf[256];
      std::strncpy(buf, s.c_str(), sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';
      if(ImGui::InputText(f.name, buf, sizeof(buf))) {
        props->set<std::string>(f.name, std::string(buf));
        changed = true;
      }
    }
  }
  return changed;
}

} // namespace mu
