#include <algorithm>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

void BrushSizePanel::Update() {
  if(ImGui::Begin(paint_window_title("brush_size").c_str(), &open)) {
    static constexpr int kSizes[] = {1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 20, 25, 30, 40, 50, 60, 70, 80, 100, 120, 150, 170, 200, 250, 300, 400, 500, 600, 800, 1000, 1500, 2000};
    BrushPreset* p                = paint_editor().preset();
    ImGui::BeginDisabled(!p);
    const float line = ImGui::GetTextLineHeight();
    const float dot  = 20.0f; // 丸を描く部分の高さ
    const ImVec2 cell(32.0f, dot + line);
    for(size_t i = 0; i < std::size(kSizes); i++) {
      if(i) flow_same_line(cell.x);
      const int size = kSizes[i];
      ImGui::PushID(size);
      const bool selected = p && p->current.size == (float)size;
      const ImVec2 min    = ImGui::GetCursorScreenPos();
      if(ImGui::Selectable("##size", selected, 0, cell) && p) p->current.size = (float)size;
      // 丸は実寸(大きいものは頭打ち)、下に数値
      auto* dl            = ImGui::GetWindowDrawList();
      const ImU32 col     = ImGui::GetColorU32(ImGuiCol_Text);
      const auto text     = std::to_string(size);
      const float radius  = std::clamp(size * 0.5f, 0.75f, dot * 0.5f - 1.0f);
      const float text_w  = ImGui::CalcTextSize(text.c_str()).x;
      const ImVec2 center = ImVec2(min.x + cell.x * 0.5f, min.y + dot * 0.5f);
      dl->AddCircleFilled(center, radius, col);
      dl->AddText(ImVec2(min.x + (cell.x - text_w) * 0.5f, min.y + dot), col, text.c_str());
      ImGui::PopID();
    }
    ImGui::EndDisabled();
  }
  ImGui::End();
}

} // namespace mu
