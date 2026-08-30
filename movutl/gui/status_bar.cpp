#include <IconsFontAwesome6.h>
#include <filesystem>
#include <movutl/app/export_state.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {

// 画面下部に固定表示されるステータスバー(プロジェクト名称とFPSを表示)。
// ドッキング・移動不可の固定フッターとして動作する。
void render_status_bar() {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float height            = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));

  constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus;
  if(!ImGui::Begin("##Status Bar", nullptr, flags)) {
    ImGui::End();
    return;
  }

  const auto pj                = Project::Get();
  const std::string proj_name  = (pj && !pj->path.empty()) ? std::filesystem::path(pj->path).filename().string() : "(無題)";
  const float fps              = ImGui::GetIO().Framerate;
  const bool has_fps           = fps > 0.0f;
  const float right_area_width = 200.0f;

  ImGui::Text(ICON_FA_FILE " %s", proj_name.c_str());

  if(is_exporting()) {
    auto& prog      = get_export_progress();
    const int total = prog.total_frames.load();
    const int done  = prog.current_frame.load();
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), ICON_FA_FILE_EXPORT " 出力中 %d / %d", done, total);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::ProgressBar(total > 0 ? (float)done / (float)total : 1.0f, ImVec2(120.0f, 0.0f));
  }

  if(has_fps) {
    const float ms = 1000.0f / fps;
    ImGui::SameLine(ImGui::GetWindowWidth() - right_area_width);
    ImGui::Text("%.1f FPS (%.2f ms)", fps, ms);
  }
  ImGui::End();
}

} // namespace mu
