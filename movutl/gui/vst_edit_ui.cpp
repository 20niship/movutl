#include <imgui.h>
#include <movutl/gui/vst_edit_ui.hpp>
#include <string>
#include <uapmd-plugin-hosting/uapmd-plugin-hosting.hpp>
#include <unordered_map>

namespace mu {

namespace {
// ponytail: プラグインインスタンス単位でグローバルに開閉状態を持つ(単一ウィンドウ限定なので十分)。破棄されたインスタンスのエントリは残り続けるが、キーはポインタ値のみで実害は無い
std::unordered_map<void*, bool> generic_ui_open_;
} // namespace

void draw_vst_edit_button(const char* label_id, uapmd_plugin_hosting::AudioPluginInstanceAPI* inst) {
  ImGui::PushID(label_id);
  ImGui::BeginDisabled(inst == nullptr);
  if(ImGui::Button("Edit")) {
    if(inst->hasUISupport()) {
      if(!inst->isUIVisible()) {
        inst->createUI(true, nullptr, [](uint32_t, uint32_t) { return true; });
        inst->showUI();
      }
    } else {
      generic_ui_open_[inst] = true;
    }
  }
  ImGui::EndDisabled();

  if(inst != nullptr && generic_ui_open_[inst]) {
    bool open = true;
    ImGui::SetNextWindowSize(ImVec2(320, 240), ImGuiCond_FirstUseEver);
    if(ImGui::Begin((std::string("VSTパラメータ##") + label_id).c_str(), &open)) {
      for(auto& pm : inst->parameterMetadataList()) {
        float v = (float)inst->getParameterValue((int32_t)pm.index);
        ImGui::PushID((int)pm.index);
        if(ImGui::SliderFloat(pm.name.c_str(), &v, (float)pm.minPlainValue, (float)pm.maxPlainValue)) inst->setParameterValue((int32_t)pm.index, (double)v);
        ImGui::PopID();
      }
    }
    ImGui::End();
    generic_ui_open_[inst] = open;
  }
  ImGui::PopID();
}

} // namespace mu
