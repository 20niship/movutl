#include <doctest/doctest.h>
#include <movutl/gui/gui.hpp>

using namespace mu;

TEST_CASE("Workspace add_entry") {
  Workspace ws;
  ws.add_entry("Viewer", ImGuiDir_Left, 0.75f);
  REQUIRE(ws.entries.size() == 1);
  CHECK(ws.entries[0].window_name == "Viewer");
  CHECK(ws.entries[0].dir == ImGuiDir_Left);
  CHECK(ws.entries[0].ratio == doctest::Approx(0.75f));

  ws.add_entry("Timeline", ImGuiDir_Down, 0.4f).add_entry("Inspector", ImGuiDir_Right, 0.25f);
  CHECK(ws.entries.size() == 3);
  CHECK(ws.entries[2].window_name == "Inspector");

  ws.clear_entries();
  CHECK(ws.entries.empty());
}

TEST_CASE("register_workspace / remove_workspace") {
  Workspace ws;
  ws.add_entry("Viewer", ImGuiDir_None, 1.0f);
  register_workspace("test_ws", ws);

  auto app = detail::AppMain::Get();
  REQUIRE(app->workspaces.contains("test_ws"));
  CHECK(app->workspaces["test_ws"].name == "test_ws");
  REQUIRE(app->workspaces["test_ws"].entries.size() == 1);

  remove_workspace("test_ws");
  CHECK_FALSE(app->workspaces.contains("test_ws"));
}

TEST_CASE("apply_workspace 遅延適用") {
  // フレーム開始前(dockspace_id == 0)の場合、pending_workspaceに登録される
  GUIManager::Get()->dockspace_id = 0;
  Workspace ws;
  ws.add_entry("Viewer", ImGuiDir_None, 1.0f);
  register_workspace("deferred_ws", ws);

  apply_workspace("deferred_ws");
  CHECK(GUIManager::Get()->pending_workspace == "deferred_ws");

  // 存在しないワークスペースは無視される
  GUIManager::Get()->pending_workspace.clear();
  apply_workspace("not_registered");
  CHECK(GUIManager::Get()->pending_workspace.empty());

  remove_workspace("deferred_ws");
}
