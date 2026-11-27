// ウィンドウを開かずにImGuiのフレームを回し、ワークスペースの切替・保存・初期化を確認する
#include <doctest/doctest.h>
#include <filesystem>
#include <imgui.h>
#include <imgui_internal.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/gui/workspace_presets.hpp>

using namespace mu;
using namespace mu::detail;

namespace {

ImGuiContext* new_context() {
  auto ctx = ImGui::CreateContext();
  ImGui::SetCurrentContext(ctx);
  auto& io       = ImGui::GetIO();
  io.ConfigFlags = ImGuiConfigFlags_DockingEnable;
  io.DisplaySize = ImVec2(1280, 720);
  io.IniFilename = "imgui_wstest.ini";
  unsigned char* pixels;
  int w, h;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
  init_workspace_settings();
  return ctx;
}

// gui_new_frame()とupdate_gui_panels()のワークスペース関連部分だけを再現する
void frames() {
  auto gui = GUIManager::Get();
  for(int i = 0; i < 6; i++) {
    switch_pending_workspace();
    ImGui::NewFrame();
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("host", nullptr, ImGuiWindowFlags_NoDocking);
    const bool first  = gui->dockspace_id == 0;
    gui->dockspace_id = ImGui::DockSpace(ImGui::GetID("MainDockSpace"));
    ImGui::End();
    if(first) init_first_frame_layout();
    if(gui->rebuild_layout) {
      gui->rebuild_layout = false;
      build_workspace_layout();
    }
    for(const char* name : {"wstest_side", "wstest_main", "wstest_other", "wstest_new_tab"}) {
      if(!is_window_in_workspace(name)) continue;
      if(i == 0 && std::string(name) == "wstest_new_tab") continue; // 1フレーム遅れて初めて作られるタブ
      ImGui::Begin(name);
      ImGui::End();
    }
    ImGui::Render();
  }
}

struct DummyPanel final : UIPanel {
  void Update() override {}
};

bool same_node(const char* a, const char* b) { return ImGui::FindWindowByName(a)->DockId == ImGui::FindWindowByName(b)->DockId; }

} // namespace

TEST_CASE("ワークスペースの配置は切替と再起動をまたいで残り、初期配置に戻せる") {
  auto gui            = GUIManager::Get();
  auto prev_ctx       = ImGui::GetCurrentContext();
  const auto prev_id  = gui->dockspace_id;
  const auto prev_cur = gui->current_workspace;

  Workspace a, b;
  a.add_entry("wstest_side", ImGuiDir_Left, 0.3f).add_entry("wstest_main", ImGuiDir_None, 1.0f);
  b.add_entry("wstest_other", ImGuiDir_Left, 0.5f).split_entry("wstest_new_tab", "wstest_other", ImGuiDir_None, 0.0f).add_entry("wstest_rest", ImGuiDir_None, 1.0f);
  register_workspace("wstest_a", a);
  register_workspace("wstest_b", b);

  auto ctx               = new_context();
  gui->dockspace_id      = 0;
  gui->current_workspace = "wstest_a";
  gui->rebuild_layout    = true;
  frames();
  CHECK_FALSE(same_node("wstest_side", "wstest_main"));

  const auto docked = [] { return ImGui::FindWindowByName("wstest_side")->DockId != 0; };
  REQUIRE(docked());

  // ユーザーによる配置変更(ドックから切り離す)
  ImGui::DockBuilderDockWindow("wstest_side", 0);
  frames();
  REQUIRE_FALSE(docked());

  apply_workspace("wstest_b");
  frames();
  CHECK(gui->current_workspace == "wstest_b");
  auto other = ImGui::FindWindowByName("wstest_other");
  REQUIRE(other->DockNode);
  REQUIRE(other->DockNode->TabBar);
  CHECK(other->DockNode->TabBar->VisibleTabId == other->TabId); // タブは最初に登録したものが選択される
  apply_workspace("wstest_a");
  frames();
  CHECK_FALSE(docked());

  // 保存した配置に戻っても、選択していたタブは変わらない
  apply_workspace("wstest_b");
  frames();
  other = ImGui::FindWindowByName("wstest_other");
  REQUIRE(other->DockNode);
  REQUIRE(other->DockNode->TabBar);
  CHECK(other->DockNode->TabBar->VisibleTabId == other->TabId);
  apply_workspace("wstest_a");
  frames();

  // 再起動: imgui.iniから現在のワークスペースと配置が戻る
  ImGui::DestroyContext(ctx);
  gui->dockspace_id      = 0;
  gui->current_workspace = "Default";
  ctx                    = new_context();
  frames();
  CHECK(gui->current_workspace == "wstest_a");
  CHECK_FALSE(docked());

  reset_workspace();
  frames();
  CHECK(docked());
  CHECK_FALSE(same_node("wstest_side", "wstest_main"));

  // iniに残ったワークスペースが未登録ならDefaultを組み直す
  const auto workspaces  = &AppMain::Get()->workspaces;
  const bool had_default = workspaces->contains("Default");
  if(!had_default) register_workspace("Default", a);
  gui->current_workspace = "wstest_removed";
  ImGui::SaveIniSettingsToDisk("imgui_wstest.ini");
  ImGui::DestroyContext(ctx);
  gui->dockspace_id      = 0;
  gui->current_workspace = "Default";
  ctx                    = new_context();
  frames();
  CHECK(gui->current_workspace == "Default");
  CHECK_FALSE(gui->rebuild_layout);
  if(!had_default) remove_workspace("Default");

  // 初期配置に戻すと、閉じたウィンドウも開き直す(サブツール詳細は除く)
  const auto panels = gui->panels;
  gui->panels.clear();
  set_paint_panel("wstest_closed", cutil::make_ref<DummyPanel>());
  set_paint_panel("brush_detail", cutil::make_ref<DummyPanel>());
  for(auto& p : gui->panels) p->open = false;
  reset_workspace();
  CHECK(gui->panels[0]->open);
  CHECK_FALSE(gui->panels[1]->open);
  gui->panels         = panels;
  gui->rebuild_layout = false;

  ImGui::DestroyContext(ctx);
  ImGui::SetCurrentContext(prev_ctx);
  gui->dockspace_id      = prev_id;
  gui->current_workspace = prev_cur;
  remove_workspace("wstest_a");
  remove_workspace("wstest_b");
  for(const char* f : {"imgui_wstest.ini", "imgui_wstest_a.ini", "imgui_wstest_b.ini"}) std::filesystem::remove(f);
}
