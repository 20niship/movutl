#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/core/command.hpp>
#include <movutl/gui/commands_input.hpp>
#include <movutl/gui/workspace_presets.hpp>

using namespace mu;
using namespace mu::detail;

TEST_CASE("parse_shortcut 記号キーと修飾キー") {
  const std::pair<const char*, ImGuiKey> keys[] = {{"[", ImGuiKey_LeftBracket}, {"]", ImGuiKey_RightBracket}, {"/", ImGuiKey_Slash}, {"+", ImGuiKey_Equal}, {"-", ImGuiKey_Minus}, {",", ImGuiKey_Comma}, {".", ImGuiKey_Period}, {";", ImGuiKey_Semicolon}};
  for(const auto& [name, key] : keys) {
    CAPTURE(name);
    const auto c = parse_shortcut(std::string("ctrl+") + name);
    REQUIRE(c.size() == 1);
    CHECK(c[0].ctrl);
    CHECK_FALSE(c[0].shift);
    CHECK(c[0].key == key);
    CHECK(parse_shortcut(name)[0].key == key);
  }
  // "+"はShiftの有無を問わない
  CHECK(parse_shortcut("+")[0].matches({false, true, false, false, ImGuiKey_Equal}));
  CHECK_FALSE(parse_shortcut("=")[0].matches({false, true, false, false, ImGuiKey_Equal}));
  CHECK(normalize_key(ImGuiKey_KeypadAdd) == ImGuiKey_Equal);

  const auto alt = parse_shortcut("alt");
  CHECK(alt[0].alt);
  CHECK(alt[0].key == ImGuiKey_None);

  // 未知のキー名は修飾キーのみの割り当てにしない
  const auto bad = parse_shortcut("ctrl+nokey");
  CHECK_FALSE(bad[0].ctrl);
  CHECK(bad[0].key == ImGuiKey_None);

  CHECK(parse_shortcut("g g").size() == 2);
}

TEST_CASE("find_shortcut_command contextの優先順位") {
  auto info = [](const char* id, const char* shortcut, const char* context) {
    CommandInfo i;
    i.id       = id;
    i.shortcut = shortcut;
    i.context  = context;
    return i;
  };
  const std::vector<CommandInfo> infos = {info("global", "ctrl+d", ""), info("canvas", "ctrl+d", "canvas"), info("timeline_only", "delete", "timeline")};
  bool prefix                          = false;

  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+d"), "canvas", prefix)->id == "canvas");
  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+d"), "", prefix)->id == "global");
  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+d"), "timeline", prefix)->id == "global");
  CHECK(find_shortcut_command(infos, parse_shortcut("delete"), "canvas", prefix) == nullptr);
  CHECK_FALSE(prefix);
}

TEST_CASE("find_shortcut_command ペイント系ワークスペースのコマンドは全体より優先、contextより後") {
  auto info = [](const char* id, const char* context) {
    CommandInfo i;
    i.id       = id;
    i.shortcut = "ctrl+s";
    i.context  = context;
    return i;
  };
  const std::vector<CommandInfo> infos = {info("global", ""), info("paint", "paint"), info("canvas", "canvas")};
  bool prefix                          = false;
  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+s"), "", prefix)->id == "global");
  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+s"), "", prefix, "paint")->id == "paint");
  CHECK(find_shortcut_command(infos, parse_shortcut("ctrl+s"), "canvas", prefix, "paint")->id == "canvas");
}

TEST_CASE("paint_window_title / ワークスペース内のウィンドウ判定") {
  auto gui               = GUIManager::Get();
  const auto before      = gui->current_workspace;
  gui->current_workspace = "Illust";
  CHECK(paint_window_title("subtool") == "サブツール###subtool");
  gui->current_workspace = "Photo";
  CHECK(paint_window_title("subtool") == "ブラシ###subtool");

  Workspace video, paint;
  video.add_entry("test_video", ImGuiDir_None, 1.0f);
  paint.add_entry("test_paint", ImGuiDir_None, 1.0f);
  register_workspace("test_video_ws", video);
  register_workspace("test_paint_ws", paint);
  gui->current_workspace = "test_paint_ws";
  CHECK(is_window_in_workspace("test_paint"));
  CHECK_FALSE(is_window_in_workspace("test_video"));
  CHECK(is_window_in_workspace("test_user_window"));
  CHECK(is_window_in_workspace(""));

  remove_workspace("test_video_ws");
  remove_workspace("test_paint_ws");
  gui->current_workspace = before;
}

TEST_CASE("動画編集の編集系ショートカットはペイント系ワークスペースで効かない") {
  register_default_commands();
  bool prefix = false;
  for(const char* key : {"s", "shift+p"}) {
    CAPTURE(key);
    const auto* video = find_shortcut_command(get_command_infos(), parse_shortcut(key), "", prefix, "video");
    REQUIRE(video);
    CHECK(video->context == "video");
    const auto* paint = find_shortcut_command(get_command_infos(), parse_shortcut(key), "canvas", prefix, "paint");
    CHECK((paint == nullptr || paint->context != "video"));
  }
  // フレーム送りとundoはどちらでも効く
  for(const char* key : {"left", "right", "ctrl+z"}) {
    CAPTURE(key);
    CHECK(find_shortcut_command(get_command_infos(), parse_shortcut(key), "", prefix, "paint"));
    CHECK(find_shortcut_command(get_command_infos(), parse_shortcut(key), "", prefix, "video"));
  }
}
