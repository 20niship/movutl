#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cctype>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {
namespace {

const char* tool_icon(const std::string& id) {
  if(id == "pen") return ICON_FA_PEN_NIB;
  if(id == "pencil") return ICON_FA_PENCIL;
  if(id == "brush") return ICON_FA_PAINTBRUSH;
  if(id == "airbrush") return ICON_FA_SPRAY_CAN;
  if(id == "deco") return ICON_FA_WAND_MAGIC_SPARKLES;
  if(id == "eraser") return ICON_FA_ERASER;
  if(id == "eyedropper") return ICON_FA_EYE_DROPPER;
  return ICON_FA_QUESTION;
}

} // namespace

void ToolPanel::Update() {
  // 細い列で切れないように余白を詰める
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
  const bool visible = ImGui::Begin(paint_window_title("paint_tool").c_str(), &open);
  ImGui::PopStyleVar();
  if(visible) {
    auto& st         = paint_editor();
    const float size = std::clamp(ImGui::GetContentRegionAvail().x, 24.0f, 36.0f);
    bool first       = true;
    for(const auto& t : st.tools.tools()) {
      if(!first) flow_same_line(size);
      first               = false;
      const bool selected = &st.tools.active() == t.get();
      if(selected) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
      if(ImGui::Button((std::string(tool_icon(t->id)) + "##" + t->id).c_str(), ImVec2(size, size))) st.tools.select(t->id);
      if(selected) ImGui::PopStyleColor();
      std::string key = t->key;
      for(char& c : key) c = (char)std::toupper((unsigned char)c);
      ImGui::SetItemTooltip("%s (%s)", t->name.c_str(), key.c_str());
    }
  }
  ImGui::End();
}

} // namespace mu
