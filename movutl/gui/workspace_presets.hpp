#pragma once
#include <movutl/gui/gui.hpp>
#include <string>

namespace mu {

// ペイント系ウィンドウのImGuiウィンドウ名 "表示名###id"。表示名は現在のワークスペース(クリスタ式 / Photoshop式)で変わる
std::string paint_window_title(const std::string& id);

// idのプレースホルダを実パネルに差し替える(detail::init_workspaces()より後に呼ぶ)。open状態は引き継ぐ
// 実パネルはImGui::Begin(paint_window_title(id).c_str(), &open)でウィンドウを作ること
void set_paint_panel(const std::string& id, Ref<UIPanel> panel);

// メニューに出すワークスペースの表示名
std::string workspace_label(const std::string& name);

namespace detail {

void init_workspaces(); // 4つのワークスペースとペイント系のプレースホルダパネルを登録する
bool is_window_in_workspace(const std::string& window_name);
bool is_paint_workspace(); // 現在のワークスペースがキャンバスを持つ(Ctrl+S等の対象がキャンバスになる)

void init_workspace_settings();  // 現在のワークスペース名をimgui.iniに保存するハンドラを登録する
void switch_pending_workspace(); // 毎フレームImGui::NewFrame()の前に呼ぶ
void build_workspace_layout();   // 現在のワークスペースの初期配置を組む(DockSpace生成後に呼ぶ)
void init_first_frame_layout();  // 初フレームのDockSpace生成後に呼ぶ。保存済みの配置を使うか、組み直すかを決める

} // namespace detail
} // namespace mu
