#include <IconsFontAwesome6.h>
#include <algorithm>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {
namespace {

// 描画色の切替ボタン。透明色(c==nullptr)は市松模様
bool slot_button(const char* id, const Vec4b* c, bool selected, float size) {
  const ImVec2 min   = ImGui::GetCursorScreenPos();
  const bool clicked = ImGui::InvisibleButton(id, ImVec2(size, size));
  const ImVec2 max(min.x + size, min.y + size);
  auto* dl = ImGui::GetWindowDrawList();
  if(c) {
    dl->AddRectFilled(min, max, ImGui::GetColorU32(to_imvec4(*c)));
  } else {
    const float h = size * 0.25f;
    for(int y = 0; y < 4; y++)
      for(int x = 0; x < 4; x++) dl->AddRectFilled(ImVec2(min.x + x * h, min.y + y * h), ImVec2(min.x + (x + 1) * h, min.y + (y + 1) * h), (x + y) % 2 ? IM_COL32(150, 150, 150, 255) : IM_COL32(235, 235, 235, 255));
  }
  // 選択中は目立つ二重枠
  if(selected) {
    dl->AddRect(ImVec2(min.x - 2, min.y - 2), ImVec2(max.x + 2, max.y + 2), IM_COL32(255, 170, 0, 255), 0, 0, 3.0f);
  } else {
    dl->AddRect(min, max, IM_COL32(128, 128, 128, 255));
  }
  return clicked;
}

Vec4b to_color(const float* rgb) {
  const auto b = [](float v) { return (unsigned char)(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
  return Vec4b{b(rgb[0]), b(rgb[1]), b(rgb[2]), 255};
}

} // namespace

void ColorPanel::Update() {
  if(!ImGui::Begin(paint_window_title("color").c_str(), &open)) {
    ImGui::End();
    return;
  }
  auto& st = paint_editor();

  // 外(スポイトやキー)で色が変わった時だけ取り込む
  const Vec4b cur = st.color();
  const Vec4b own = to_color(rgb_);
  if(own[0] != cur[0] || own[1] != cur[1] || own[2] != cur[2])
    for(int i = 0; i < 3; i++) rgb_[i] = cur[i] / 255.0f;

  // 左にカラーサークル、右に切替ボタンを縦に並べる(低いパレットでも両方見えるように)
  const float chip   = 24.0f;
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float side   = std::clamp(std::min(avail.x - chip - 16.0f, avail.y - ImGui::GetFrameHeightWithSpacing()), 90.0f, 320.0f);
  ImGui::SetNextItemWidth(side);
  if(ImGui::ColorPicker3("##wheel", rgb_, ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview | ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_DisplayHSV)) st.set_color(to_color(rgb_));

  ImGui::SameLine(0, 10);
  ImGui::BeginGroup();
  if(slot_button("##main", &st.main_color, st.color_slot == PaintColorSlot::Main, chip)) st.color_slot = PaintColorSlot::Main;
  ImGui::SetItemTooltip("メインカラー (X でサブカラーと切替)");
  if(slot_button("##sub", &st.sub_color, st.color_slot == PaintColorSlot::Sub, chip)) st.color_slot = PaintColorSlot::Sub;
  ImGui::SetItemTooltip("サブカラー (X でメインカラーと切替)");
  if(slot_button("##transparent", nullptr, st.color_slot == PaintColorSlot::Transparent, chip) && st.color_slot != PaintColorSlot::Transparent) st.toggle_transparent();
  ImGui::SetItemTooltip("透明色 (C)");
  if(ImGui::Button(ICON_FA_RIGHT_LEFT "##swap", ImVec2(chip, chip))) st.swap_color();
  ImGui::SetItemTooltip("メインカラーとサブカラーを切り替え (X)");
  ImGui::EndGroup();

  if(!st.color_history.empty()) {
    ImGui::SeparatorText("カラーヒストリー");
    const float s = 18.0f;
    for(size_t i = 0; i < st.color_history.size(); i++) {
      if(i) flow_same_line(s);
      const Vec4b c = st.color_history[i];
      ImGui::PushID((int)i);
      if(ImGui::ColorButton("##history", to_imvec4(c), ImGuiColorEditFlags_NoAlpha, ImVec2(s, s))) st.set_color(c);
      ImGui::PopID();
    }
  }
  ImGui::End();
}

} // namespace mu
