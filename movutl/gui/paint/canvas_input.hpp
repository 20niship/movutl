#pragma once

#include <movutl/input/pen_input.hpp>
#include <vector>

struct GLFWwindow;

namespace mu {

struct CanvasInputEvent {
  PenEvent event; // 座標はImGuiのスクリーン座標(ウィンドウのクライアント座標)
  bool pen = false;
};

unsigned long x11_window_id(GLFWwindow* window); // X11でなければ0

// ウィンドウ作成後に1回呼ぶ。ペンが使えない環境ではマウスだけになる
void open_pen_input(GLFWwindow* window);
// 毎フレームImGui::NewFrame()の後に呼ぶ。キャンバスが非表示でも呼び、入力を溜め込まない
void poll_canvas_input();
// 直近のpoll_canvas_input()で届いた入力。ペン操作中に重ねて届くマウスのDownは含まない
const std::vector<CanvasInputEvent>& canvas_input_events();

} // namespace mu
