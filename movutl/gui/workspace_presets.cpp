#include <IconsFontAwesome6.h>
#include <imgui.h>
// --
#include <algorithm>
#include <movutl/app/app_impl.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {
namespace {

struct PaintWindow {
  const char* id;
  const char* csp;       // クリスタ式の表示名
  const char* photoshop; // Photoshop式の表示名(画像加工)
};

// 同じノードのタブは描画順に並ぶので、裏に回るタブ(history, adjust等)を後に並べる
constexpr PaintWindow kPaintWindows[] = {
  {"canvas", "キャンバス", "キャンバス"},
  {"paint_tool", "ツール", "ツール"},
  {"color", "カラーサークル", "カラー"},
  {"tool_prop", "ツールプロパティ", "オプションバー"},
  {"brush_size", "ブラシサイズ", "ブラシサイズ"},
  {"layers", "レイヤー", "レイヤー"},
  {"layer_prop", "レイヤープロパティ", "プロパティ"},
  {"subtool", "サブツール", "ブラシ"},
  {"navigator", "ナビゲーター", "ナビゲーター / 情報"},
  {"history", "ヒストリー", "ヒストリー"},
  {"adjust", "色調補正", "色調補正"},
  {"brush_detail", "サブツール詳細", "ブラシ設定"},
  {"command_bar", "コマンドバー", "コマンドバー"},
  {"cel_timeline", "タイムライン", "タイムライン"},
  {"anim_cel", "アニメーションセル", "アニメーションセル"},
};

// 実装前のパネル
struct PlaceholderPanel final : UIPanel {
  explicit PlaceholderPanel(std::string id) : id_(std::move(id)) {}
  void Update() override {
    if(ImGui::Begin(paint_window_title(id_).c_str(), &open)) ImGui::TextDisabled("未実装 (%s)", id_.c_str());
    ImGui::End();
  }
  std::string id_;
};

// イラストとアニメ(クリスタ式)。左にツールとサブツール類、右にナビゲーターとレイヤー、中央にキャンバス
Workspace clip_studio_workspace(bool anime) {
  Workspace ws;
  ws.add_entry("###command_bar", ImGuiDir_Up, 0.06f);
  if(anime) ws.add_entry("###cel_timeline", ImGuiDir_Down, 0.22f);
  ws.add_entry("###paint_tool", ImGuiDir_Left, 0.05f);
  ws.add_entry("###subtool", ImGuiDir_Left, 0.19f);
  ws.split_entry("###tool_prop", "###subtool", ImGuiDir_Down, 0.76f);
  ws.split_entry("###brush_size", "###tool_prop", ImGuiDir_Down, 0.56f);
  ws.split_entry("###color", "###brush_size", ImGuiDir_Down, 0.7f);
  ws.add_entry("###navigator", ImGuiDir_Right, 0.24f);
  const char* middle = anime ? "###anim_cel" : "###layer_prop";
  ws.split_entry(middle, "###navigator", ImGuiDir_Down, 0.66f);
  ws.split_entry("###layers", middle, ImGuiDir_Down, anime ? 0.72f : 0.8f);
  ws.split_entry("###history", "###layers", ImGuiDir_None, 0.0f);
  ws.add_entry("###brush_detail", ImGuiDir_None, -1.0f);
  ws.add_entry("###canvas", ImGuiDir_None, 1.0f);
  return ws;
}

// 画像加工(Photoshop式)。上にオプションバー、左にツール、右にカラー / プロパティ / レイヤー
Workspace photoshop_workspace() {
  Workspace ws;
  ws.add_entry("###tool_prop", ImGuiDir_Up, 0.1f);
  ws.add_entry("###paint_tool", ImGuiDir_Left, 0.05f);
  ws.add_entry("###color", ImGuiDir_Right, 0.24f);
  // Photoshopではアイコン列に畳まれるパレットは、カラーのタブにする
  ws.split_entry("###subtool", "###color", ImGuiDir_None, 0.0f);
  ws.split_entry("###history", "###color", ImGuiDir_None, 0.0f);
  ws.split_entry("###navigator", "###color", ImGuiDir_None, 0.0f);
  ws.split_entry("###layer_prop", "###color", ImGuiDir_Down, 0.7f);
  ws.split_entry("###adjust", "###layer_prop", ImGuiDir_None, 0.0f);
  ws.split_entry("###layers", "###layer_prop", ImGuiDir_Down, 0.65f);
  ws.add_entry("###brush_detail", ImGuiDir_None, -1.0f);
  ws.add_entry("###canvas", ImGuiDir_None, 1.0f);
  return ws;
}

} // namespace

std::string paint_window_title(const std::string& id) {
  const bool photoshop = GUIManager::Get()->current_workspace == "Photo";
  for(const auto& w : kPaintWindows)
    if(id == w.id) return std::string(photoshop ? w.photoshop : w.csp) + "###" + id;
  return id + "###" + id;
}

void set_paint_panel(const std::string& id, Ref<UIPanel> panel) {
  panel->window_name = "###" + id;
  for(auto& p : GUIManager::Get()->panels) {
    if(p->window_name != panel->window_name) continue;
    panel->open = p->open;
    p           = panel;
    return;
  }
  GUIManager::Get()->panels.push_back(panel);
}

std::string workspace_label(const std::string& name) {
  if(name == "Default") return "動画編集";
  if(name == "Illust") return "イラスト";
  if(name == "Anime") return "アニメ";
  if(name == "Photo") return "画像加工";
  return name;
}

namespace detail {

bool is_paint_workspace() {
  auto& workspaces = AppMain::Get()->workspaces;
  const auto it    = workspaces.find(GUIManager::Get()->current_workspace);
  return it != workspaces.end() && std::any_of(it->second.entries.begin(), it->second.entries.end(), [](const auto& e) { return e.window_name == "###canvas"; });
}

void init_workspaces() {
  // 動画編集(初回起動時に適用される)。dir==Noneのentry(最後を除く)は直前entryと同タブになる
  Workspace video;
  video.add_entry("タイムライン", ImGuiDir_Down, 0.36f);
  video.add_entry(ICON_FA_CHART_LINE " グラフエディタ", ImGuiDir_None, 0.0f);
  video.add_entry("ツール", ImGuiDir_Left, 0.17f);
  video.add_entry(ICON_FA_PLUG " エフェクト制御", ImGuiDir_Right, 0.32f);
  video.add_entry(ICON_FA_KEYBOARD " ピアノロール", ImGuiDir_Down, 0.5f);
  video.add_entry("Viewer", ImGuiDir_None, 1.0f);
  register_workspace("Default", video);
  register_workspace("Illust", clip_studio_workspace(false));
  register_workspace("Anime", clip_studio_workspace(true));
  register_workspace("Photo", photoshop_workspace());

  for(const auto& w : kPaintWindows) set_paint_panel(w.id, cutil::make_ref<PlaceholderPanel>(w.id));
  // サブツール詳細は必要な時に開くフローティングウィンドウ
  for(auto& p : GUIManager::Get()->panels)
    if(p->window_name == "###brush_detail") p->open = false;
}

} // namespace detail
} // namespace mu
