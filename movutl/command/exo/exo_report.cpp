#include <atomic>
#include <imgui.h>
#include <movutl/command/exo/exo_report.hpp>

namespace mu {

namespace {
ExoImportReport g_report;
std::atomic<bool> g_open_request{false};
constexpr const char* kDialogId = "EXO取り込み結果";
} // namespace

void ExoImportReport::add(const std::string& msg) {
  for(auto& it : items) {
    if(it.msg == msg) {
      ++it.count;
      return;
    }
  }
  items.push_back({msg, 1});
}

ExoImportReport& exo_import_report() { return g_report; }

void exo_import_report_begin(const std::string& path) {
  g_report      = ExoImportReport();
  g_report.path = path;
}

void exo_import_report_request_dialog() { g_open_request = true; }

void draw_exo_import_report_dialog() {
  if(g_open_request.exchange(false)) ImGui::OpenPopup(kDialogId);
  if(!ImGui::BeginPopupModal(kDialogId, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::Text("%d 個のオブジェクトを取り込みました", g_report.imported);
  ImGui::TextDisabled("%s", g_report.path.c_str());
  ImGui::Separator();
  ImGui::Text("反映できなかった項目(%d 件)", (int)g_report.items.size());
  ImGui::BeginChild("##exo_report_items", ImVec2(560, 220), true);
  for(auto& it : g_report.items) {
    if(it.count > 1)
      ImGui::BulletText("%s (x%d)", it.msg.c_str(), it.count);
    else
      ImGui::BulletText("%s", it.msg.c_str());
  }
  ImGui::EndChild();
  if(ImGui::Button("OK", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

} // namespace mu
