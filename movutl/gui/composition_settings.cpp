#include <cstring>
#include <imgui.h>
#include <movutl/asset/project.hpp>
#include <movutl/gui/composition_settings.hpp>

namespace mu {

void CompositionSettingsWindow::Update() {
  auto pj = Project::Get();
  for(auto& cmp : pj->compos_) {
    if(!(cmp.flag & Composition::setting_dialog)) continue;

    bool open = true;
    char title[128];
    std::snprintf(title, sizeof(title), "Composition Settings##%u", cmp.guid);
    if(ImGui::Begin(title, &open)) {
      char name_buf[128];
      std::snprintf(name_buf, sizeof(name_buf), "%s", cmp.name.c_str());
      if(ImGui::InputText("名前", name_buf, sizeof(name_buf))) cmp.name = name_buf;

      int size2[2] = {(int)cmp.size[0], (int)cmp.size[1]};
      if(ImGui::DragInt2("解像度", size2, 1.0f, 1, 8192)) cmp.resize(size2[0], size2[1]);

      ImGui::DragFloat("フレームレート", &cmp.framerate, 0.1f, 1.0f, 1000.0f);
      ImGui::DragInt("開始フレーム", &cmp.fstart);
      ImGui::DragInt("終了フレーム", &cmp.fend);
    }
    ImGui::End();

    if(!open) cmp.flag = (Composition::Flag)(cmp.flag & ~Composition::setting_dialog);
  }
}

} // namespace mu
