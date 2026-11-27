#include <imgui.h>
// --
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <movutl/app/app.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/command.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/platform.hpp>
#include <movutl/core/status_log.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/paint/document_ui.hpp>
#include <movutl/gui/paint/editor_state.hpp>
#include <movutl/gui/workspace_presets.hpp>

#include <GLFW/glfw3.h>

namespace mu {

namespace {

namespace fs = std::filesystem;
using Action = std::function<void()>;

enum class Dialog { None, NewSize, Path, Confirm };
Dialog requested = Dialog::None; // 次のdraw_document_dialogs()で開く

int new_size[2] = {1920, 1080};

char path_buf[1024] = "";
bool path_for_save  = false;
bool overwrite_ok   = false; // 上書きの確認済み
std::string overwrite_path;
std::function<void(const std::string&)> on_path;

std::string confirm_message;
Action on_save_then_continue; // 空なら「保存して続行」を出さない
Action on_continue;
bool quit_approved = false;

std::string file_name(const std::string& path) { return fs::path(path).filename().string(); }

void choose_path(bool for_save, std::function<void(const std::string&)> next) {
#if defined(MOVUTL_PLATFORM_WINDOWS) || defined(MOVUTL_PLATFORM_MACOS)
  const std::string path = for_save ? select_save_file_dialog("名前を付けて保存", "untitled.mup", {"mup"}) : select_file_dialog("キャンバスを開く", {"mup"});
  if(!path.empty()) next(path);
#else
  // ponytail: LinuxはOSのファイルダイアログが未実装なのでパス入力。file_dialog.cppに実装されたらこの分岐を消す
  const auto& current       = paint_editor().path();
  const char* home          = std::getenv("HOME");
  const std::string initial = !current.empty() ? current : std::string(home ? home : ".") + (for_save ? "/untitled.mup" : "/");
  std::snprintf(path_buf, sizeof(path_buf), "%s", initial.c_str());
  path_for_save = for_save;
  overwrite_ok  = false;
  on_path       = std::move(next);
  requested     = Dialog::Path;
#endif
}

void write_document(const std::string& path, const Action& then) {
  if(!paint_editor().save_as(path)) {
    push_status_log(StatusLevel::Error, "保存できませんでした: " + path);
    return;
  }
  push_status_log(StatusLevel::Success, "保存しました: " + file_name(path));
  if(then) then();
}

// 保存先が未定かas_newなら保存先を聞く。thenは保存に成功した時だけ呼ぶ
bool save_document(bool as_new, Action then = {}) {
  auto& st = paint_editor();
  if(!st.doc()) return false;
  if(!as_new && !st.path().empty())
    write_document(st.path(), then);
  else
    choose_path(true, [then](const std::string& p) { write_document(p, then); });
  return true;
}

void confirm(std::string message, Action save_then_continue, Action next) {
  confirm_message       = std::move(message);
  on_save_then_continue = std::move(save_then_continue);
  on_continue           = std::move(next);
  requested             = Dialog::Confirm;
}

// 未保存のキャンバスを破棄する操作の前に確認する
void guard_document(Action next) {
  if(!paint_editor().modified()) return next();
  confirm("キャンバスの変更は保存されていません。", [next] { save_document(false, next); }, next);
}

void open_document(const std::string& path) {
  guard_document([path] {
    if(!paint_editor().open(path)) {
      push_status_log(StatusLevel::Error, "開けませんでした: " + path);
      return;
    }
    if(!detail::is_paint_workspace()) apply_workspace("Illust");
    push_status_log(StatusLevel::Success, "開きました: " + file_name(path));
  });
}

struct DocumentCommand final : mCommand {
  explicit DocumentCommand(bool (*fn)(const std::string&)) : fn_(fn) {}
  CommandStatus on_start() override { return fn_(arg) ? CommandStatus::Finished : CommandStatus::Failed; }
  bool (*fn_)(const std::string&);
};

void draw_new_size() {
  if(!ImGui::BeginPopupModal("新規キャンバス", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::InputInt2("幅 x 高さ (px)", new_size);
  constexpr int presets[][2]    = {{1920, 1080}, {2480, 3508}, {4096, 4096}, {1280, 720}};
  constexpr const char* names[] = {"1920x1080", "A4 300dpi", "4096x4096", "1280x720"};
  for(int i = 0; i < 4; i++) {
    if(i) ImGui::SameLine();
    if(ImGui::Button(names[i])) std::copy_n(presets[i], 2, new_size);
  }
  for(int& v : new_size) v = std::clamp(v, 1, PaintEditorState::kMaxCanvasSize);
  ImGui::Separator();
  if(ImGui::Button("作成") || ImGui::IsKeyPressed(ImGuiKey_Enter, false)) {
    paint_editor().new_document(new_size[0], new_size[1]);
    ImGui::CloseCurrentPopup();
  }
  ImGui::SameLine();
  if(ImGui::Button("キャンセル") || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

void draw_path() {
  if(!ImGui::BeginPopupModal("キャンバスのファイル", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::TextUnformatted(path_for_save ? "保存先のパス (.mup)" : "開くファイルのパス (.mup)");
  if(ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
  ImGui::SetNextItemWidth(520);
  const bool enter = ImGui::InputText("##path", path_buf, sizeof(path_buf), ImGuiInputTextFlags_EnterReturnsTrue);

  std::string path = path_buf;
  if(const char* home = std::getenv("HOME"); home && path.rfind("~/", 0) == 0) path = home + path.substr(1);
  if(path_for_save && !path.empty() && !fs::path(path).has_extension()) path += ".mup";
  std::error_code ec;
  const bool exists = path_for_save && path != paint_editor().path() && fs::exists(path, ec);
  if(overwrite_ok && path != overwrite_path) overwrite_ok = false;
  if(overwrite_ok) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "同じ名前のファイルがあります。もう一度押すと上書きします。");

  const char* label = !path_for_save ? "開く" : overwrite_ok ? "上書き保存" : "保存";
  if(ImGui::Button(label) || enter) {
    if(path.empty() || fs::is_directory(path, ec)) {
      push_status_log(StatusLevel::Warning, "ファイル名を指定してください");
    } else if(exists && !overwrite_ok) {
      overwrite_ok   = true;
      overwrite_path = path;
    } else {
      ImGui::CloseCurrentPopup();
      std::exchange(on_path, {})(path);
    }
  }
  ImGui::SameLine();
  if(ImGui::Button("キャンセル") || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

void draw_confirm() {
  if(!ImGui::BeginPopupModal("未保存の変更", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  ImGui::TextUnformatted(confirm_message.c_str());
  ImGui::Separator();
  Action run;
  if(on_save_then_continue) {
    if(ImGui::Button("保存して続行")) run = on_save_then_continue;
    ImGui::SameLine();
  }
  if(ImGui::Button("破棄して続行")) run = on_continue;
  ImGui::SameLine();
  const bool cancel = ImGui::Button("キャンセル") || ImGui::IsKeyPressed(ImGuiKey_Escape, false);
  if(run || cancel) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
  if(run) run();
}

} // namespace

void draw_document_dialogs() {
  switch(std::exchange(requested, Dialog::None)) {
    case Dialog::NewSize: ImGui::OpenPopup("新規キャンバス"); break;
    case Dialog::Path: ImGui::OpenPopup("キャンバスのファイル"); break;
    case Dialog::Confirm: ImGui::OpenPopup("未保存の変更"); break;
    case Dialog::None: break;
  }
  draw_new_size();
  draw_path();
  draw_confirm();
}

bool confirm_quit() {
  if(quit_approved || !paint_editor().modified()) return true;
  const Action quit = [] {
    quit_approved = true;
    glfwSetWindowShouldClose(GUIManager::Get()->glfw_window, GLFW_TRUE);
  };
  confirm("キャンバスの変更は保存されていません。終了しますか。", [quit] { save_document(false, quit); }, quit);
  return false;
}

namespace detail {

void register_document_commands() {
  const auto add = [](CommandInfo info, bool (*fn)(const std::string&)) { register_command<DocumentCommand>(std::move(info), fn); };
  add({"paint_new", "新規キャンバス", "大きさを指定して新しいキャンバスを作る", "ctrl+n", {}, "paint"}, [](const std::string&) {
    guard_document([] { requested = Dialog::NewSize; });
    return true;
  });
  add({"paint_open", "キャンバスを開く", ".mupファイルを開く", "ctrl+o", {"mup"}, "paint"}, [](const std::string& path) {
    if(path.empty())
      choose_path(false, open_document);
    else
      open_document(path);
    return true;
  });
  add({"paint_save", "キャンバスを保存", "キャンバスを.mupに保存する", "ctrl+s", {}, "paint"}, [](const std::string&) { return save_document(false); });
  add({"paint_save_as", "キャンバスに名前を付けて保存", "キャンバスを別名の.mupに保存する", "ctrl+shift+s", {}, "paint"}, [](const std::string&) { return save_document(true); });
  add({"new_project_cmd", "新規プロジェクト", "新しいプロジェクトを作成する", "ctrl+n"}, [](const std::string&) {
    // dirtyはコマンド経由の変更しか追わない(追加/移動/インスペクタ編集は漏れる)ので、中身があれば必ず確認する
    const auto* pj         = Project::Get();
    const bool has_content = !pj->entities.empty() || std::any_of(pj->compos_.begin(), pj->compos_.end(), [](const auto& c) { return !c->get_all_entities().empty(); });
    if(status_log_dirty() || has_content)
      confirm("現在のプロジェクトを破棄します。保存していない変更は失われます。", {}, new_project);
    else
      new_project();
    return true;
  });
}

} // namespace detail
} // namespace mu
