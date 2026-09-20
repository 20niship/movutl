#include <IconsFontAwesome6.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <movutl/app/app.hpp>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/status_log.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/transform_gizmo.hpp>
#include <movutl/gui/viewer.hpp>
#include <string>

namespace mu {

namespace {

ImVec4 level_color(StatusLevel lv) {
  switch(lv) {
    case StatusLevel::Success: return ImVec4(0.45f, 0.85f, 0.5f, 1.0f);
    case StatusLevel::Warning: return ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
    case StatusLevel::Error: return ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
    default: return ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
  }
}

const char* level_icon(StatusLevel lv) {
  switch(lv) {
    case StatusLevel::Success: return ICON_FA_CIRCLE_CHECK;
    case StatusLevel::Warning: return ICON_FA_TRIANGLE_EXCLAMATION;
    case StatusLevel::Error: return ICON_FA_CIRCLE_XMARK;
    default: return ICON_FA_CIRCLE_INFO;
  }
}

// フレーム番号をHH:MM:SS:FF形式にする
std::string timecode(int frame, float fps) {
  const int fps_i = std::max(1, (int)std::round(fps > 0 ? fps : 30.0f));
  const int f     = std::max(0, frame);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", f / fps_i / 3600, f / fps_i / 60 % 60, f / fps_i % 60, f % fps_i);
  return buf;
}

// 直近のログ1行。一定時間で薄くなる(エラーは残る)。クリックで履歴ポップアップを開く
void draw_latest_log() {
  constexpr double kHoldSec = 4.0, kFadeSec = 1.0;
  StatusLogEntry e;
  if(!status_log_latest(&e)) return;
  const double age = status_log_now_sec() - e.time_sec;
  float alpha      = 1.0f;
  if(e.level != StatusLevel::Error && age > kHoldSec) alpha = (float)std::clamp(1.0 - (age - kHoldSec) / kFadeSec, 0.0, 1.0);
  // フェード後も履歴を開けるよう、最低限の薄い表示は残さず領域だけクリック可能にする
  ImVec4 col             = level_color(e.level);
  col.w                  = std::max(alpha, 0.0f);
  const std::string text = std::string(level_icon(e.level)) + " " + e.message;
  ImGui::PushStyleColor(ImGuiCol_Text, col);
  ImGui::TextUnformatted(text.c_str());
  ImGui::PopStyleColor();
  if(ImGui::IsItemHovered()) {
    ImGui::SetTooltip("クリックでログ履歴を表示");
    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
  }
  if(ImGui::IsItemClicked()) ImGui::OpenPopup("##status_log_history");
}

void draw_log_history_popup() {
  ImGui::SetNextWindowSize(ImVec2(520, 260), ImGuiCond_Appearing);
  if(!ImGui::BeginPopup("##status_log_history")) return;
  const auto hist = status_log_history();
  ImGui::Text(ICON_FA_LIST " 操作ログ (%d件)", (int)hist.size());
  ImGui::SameLine();
  if(ImGui::SmallButton(ICON_FA_COPY " コピー")) {
    std::string all;
    for(const auto& h : hist) all += h.clock + "  " + h.message + "\n";
    ImGui::SetClipboardText(all.c_str());
  }
  ImGui::Separator();
  ImGui::BeginChild("##status_log_list", ImVec2(0, 0), false);
  for(auto it = hist.rbegin(); it != hist.rend(); ++it) { // 新しいものが上
    ImGui::TextDisabled("%s", it->clock.c_str());
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, level_color(it->level));
    ImGui::Text("%s %s", level_icon(it->level), it->message.c_str());
    ImGui::PopStyleColor();
  }
  ImGui::EndChild();
  ImGui::EndPopup();
}

} // namespace

// 画面下部に固定表示されるステータスバー(ドッキング・移動不可)
void render_status_bar() {
  static bool show_fps = false; // UI描画FPSは開発者向けなので既定では隠す

  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float height            = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));

  constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;
  if(!ImGui::Begin("##Status Bar", nullptr, flags)) {
    ImGui::End();
    return;
  }

  const auto pj               = Project::Get();
  const std::string proj_name = (pj && !pj->path.empty()) ? std::filesystem::path(pj->path).filename().string() : "(無題)";
  const auto* active          = Composition::GetActiveComp();

  // 左: 保存状態と名前
  const bool dirty = status_log_dirty();
  ImGui::TextColored(dirty ? ImVec4(1.0f, 0.75f, 0.3f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f), dirty ? ICON_FA_CIRCLE : ICON_FA_FILE);
  if(ImGui::IsItemHovered()) ImGui::SetTooltip(dirty ? "未保存の変更があります" : "保存済み");
  ImGui::SameLine();
  ImGui::TextUnformatted(proj_name.c_str());

  if(const auto& cur = viewer_cursor(); cur.valid && active) {
    GizmoPt p{cur.x, cur.y};
    const bool center = Config::Get()->viewer_ruler_center_origin;
    if(center) p = gizmo_to_center_origin(p, GizmoPt{(double)active->size[0], (double)active->size[1]});
    ImGui::SameLine();
    ImGui::TextDisabled(ICON_FA_LOCATION_CROSSHAIRS " X:%.0f Y:%.0f (%s原点)", p.x, p.y, center ? "中央" : "左上");
  }

  // 右側に出す項目を先に組み立てて幅を測る(中央のログ領域と重ならないように)
  std::string right;
  if(active) right += ICON_FA_CLOCK " " + timecode(active->frame, active->framerate) + " (f" + std::to_string((int)active->frame) + ")";
  const size_t nsel = get_selected_entts().size();
  if(nsel > 0) right += std::string(right.empty() ? "" : "   ") + ICON_FA_ARROW_POINTER " " + std::to_string(nsel) + "個選択";
  if(show_fps) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.1f FPS (%.2f ms)", ImGui::GetIO().Framerate, 1000.0f / std::max(1.0f, ImGui::GetIO().Framerate));
    right += std::string(right.empty() ? "" : "   ") + buf;
  }
  const float right_w = ImGui::CalcTextSize(right.c_str()).x;

  // 中央: 操作ログ(書き出し中は進捗を優先)
  ImGui::SameLine(0, 24.0f);
  if(is_exporting()) {
    auto& prog      = get_export_progress();
    const int total = prog.total_frames.load();
    const int done  = prog.current_frame.load();
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), ICON_FA_FILE_EXPORT " 出力中 %d / %d", done, total);
    ImGui::SameLine();
    ImGui::ProgressBar(total > 0 ? (float)done / (float)total : 1.0f, ImVec2(120.0f, 0.0f));
  } else {
    draw_latest_log();
  }
  draw_log_history_popup();

  // 右: フレーム/時間/選択数(FPS)。右クリックでFPS表示を切り替える
  if(!right.empty()) {
    ImGui::SameLine(ImGui::GetWindowWidth() - right_w - ImGui::GetStyle().WindowPadding.x);
    ImGui::TextUnformatted(right.c_str());
  }
  if(ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) ImGui::OpenPopup("##status_bar_ctx");
  if(ImGui::BeginPopup("##status_bar_ctx")) {
    ImGui::MenuItem("FPSを表示", nullptr, &show_fps);
    ImGui::EndPopup();
  }
  ImGui::End();
}

} // namespace mu
