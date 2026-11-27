#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/widgets.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {
namespace {

// 0〜1の値を%で編集する
bool percent_slider(float* v, int min, int max) {
  int p = (int)std::lround(*v * 100.0f);
  if(!ImGui::SliderInt("##v", &p, min, max, "%d%%", ImGuiSliderFlags_AlwaysClamp)) return false;
  *v = p / 100.0f;
  return true;
}

void angle_slider(float* rad) { ImGui::SliderAngle("##v", rad, -180.0f, 180.0f, "%.0f°", ImGuiSliderFlags_AlwaysClamp); }

// 設定項目の並び。縦長のパレットは「ラベル左・値右」の表、横長(オプションバー)は横に並べて折り返す
struct Rows {
  bool wide;
  bool first = true;
  template <class F> void operator()(const char* label, F&& widget) {
    ImGui::PushID(label);
    if(wide) {
      const float w = 110.0f;
      if(!first) flow_same_line(ImGui::CalcTextSize(label).x + ImGui::GetStyle().ItemSpacing.x + w);
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(label);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(w);
    } else {
      wd_row(label);
    }
    first = false;
    widget();
    ImGui::PopID();
  }
};

} // namespace

void ToolPropertyPanel::Update() {
  if(!ImGui::Begin(paint_window_title("tool_prop").c_str(), &open)) {
    ImGui::End();
    return;
  }
  BrushPreset* p = paint_editor().preset();
  if(!p) {
    ImGui::TextDisabled("%s: 設定項目はありません", paint_editor().tools.active().name.c_str());
    ImGui::End();
    return;
  }
  BrushSettings& s   = p->current;
  const ImVec2 area  = ImGui::GetContentRegionAvail();
  const float footer = ImGui::GetFrameHeightWithSpacing();
  Rows row{area.x > area.y * 3.0f};
  const auto slider = [](float* v, float min, float max, const char* fmt, ImGuiSliderFlags flags = 0) { ImGui::SliderFloat("##v", v, min, max, fmt, flags | ImGuiSliderFlags_AlwaysClamp); };

  // 縦長のパレットでは下端のボタン列を常に見せ、設定項目だけをスクロールする
  if(!row.wide) ImGui::BeginChild("##body", ImVec2(0, -footer));
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(p->name.c_str());
  if(row.wide) ImGui::SameLine();

  if(row.wide || wd_table_begin("##props")) {
    row("ブラシサイズ", [&] { slider(&s.size, 1.0f, PaintEditorState::kMaxBrushSize, "%.1f", ImGuiSliderFlags_Logarithmic); });
    row("硬さ", [&] { percent_slider(&s.hardness, 0, 100); });
    row("不透明度", [&] { percent_slider(&s.opacity, 1, 100); });
    row("間隔", [&] { percent_slider(&s.spacing, 1, 200); });
    row("手ブレ補正", [&] { ImGui::SliderInt("##v", &s.stabilization, 0, 100, "%d", ImGuiSliderFlags_AlwaysClamp); });
    row("筆圧: サイズ", [&] { ImGui::Checkbox("##v", &s.pressure_size); });
    row("筆圧: 不透明度", [&] { ImGui::Checkbox("##v", &s.pressure_opacity); });
    if(s.engine == "airbrush") {
      row("連続吹き付け", [&] { slider(&s.airbrush_rate, 0.0f, 10.0f, "%.1f"); });
    } else if(s.engine == "watercolor") {
      row("下地混色", [&] { percent_slider(&s.water_mix, 0, 100); });
      row("水彩境界", [&] { percent_slider(&s.edge_darkening, 0, 100); });
    } else if(s.engine == "tone") {
      row("網点の間隔", [&] { slider(&s.tone_pitch, 2.0f, 64.0f, "%.1f px"); });
      row("網点の角度", [&] { angle_slider(&s.tone_angle); });
    } else if(s.engine == "particle") {
      row("粒の数", [&] { ImGui::SliderInt("##v", &s.particle_count, 1, 100, "%d", ImGuiSliderFlags_AlwaysClamp); });
      row("粒の大きさ", [&] { percent_slider(&s.particle_size, 1, 100); });
    } else if(s.engine == "pattern") {
      row("模様の拡大率", [&] { percent_slider(&s.pattern_scale, 10, 400); });
    }
    if(!row.wide) wd_table_end();
  }

  const float button = ImGui::GetFrameHeight() * 1.3f;
  const float width  = button * 3 + ImGui::GetStyle().ItemSpacing.x * 2;
  if(row.wide) {
    flow_same_line(width);
  } else {
    ImGui::EndChild();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, ImGui::GetContentRegionAvail().x - width)); // 右寄せ
  }
  if(ImGui::Button(ICON_FA_ROTATE_LEFT "##reset", ImVec2(button, 0))) p->reset();
  ImGui::SetItemTooltip("初期設定に戻す");
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_FLOPPY_DISK "##register", ImVec2(button, 0))) ImGui::OpenPopup("初期設定に登録##confirm");
  ImGui::SetItemTooltip("初期設定に登録");
  ImGui::SameLine();
  if(ImGui::Button(ICON_FA_WRENCH "##detail", ImVec2(button, 0)))
    for(auto& panel : GUIManager::Get()->panels)
      if(panel->window_name == "###brush_detail") panel->open = true;
  ImGui::SetItemTooltip("サブツール詳細を開く");
  // 登録すると以前の初期設定は失われるので確認する
  if(ImGui::BeginPopupModal("初期設定に登録##confirm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("現在の設定を「%s」の初期設定にします。", p->name.c_str());
    if(ImGui::Button("登録")) {
      p->initial = p->current;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if(ImGui::Button("キャンセル") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  ImGui::End();
}

} // namespace mu
