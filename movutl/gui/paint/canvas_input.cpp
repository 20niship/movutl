#include <imgui.h>
// --
#include <movutl/core/logger.hpp>
#include <movutl/gui/paint/canvas_input.hpp>
#include <movutl/input/mouse_pen_input.hpp>
#include <movutl/input/xinput2_pen_input.hpp>

namespace mu {

namespace {
MousePenInput mouse;
#ifdef __linux__
XInput2PenInput pen;
#endif
std::vector<CanvasInputEvent> events;
ImVec2 mouse_pos;
} // namespace

void open_pen_input(GLFWwindow* window) {
#ifdef __linux__
  const unsigned long id = x11_window_id(window);
  if(!id || !pen.open(id)) LOG_F(INFO, "pen input is not available");
#else
  (void)window;
#endif
}

void poll_canvas_input() {
  events.clear();
  bool pen_active = false;
#ifdef __linux__
  for(const auto& e : pen.poll()) events.push_back({e, true});
  pen_active = pen.pen_active();
#endif
  // ウィンドウ外に出た瞬間はマウス座標が無効になるので、直前の座標を使う
  if(ImGui::IsMousePosValid()) mouse_pos = ImGui::GetIO().MousePos;
  mouse.update(mouse_pos.x, mouse_pos.y, ImGui::GetIO().MouseDown[0], ImGui::GetTime());
  // ペンの操作はマウスとしても届くので、その開始は捨てる(Move/Upは、マウスで始めた操作があるときだけ使われる)
  for(const auto& e : mouse.poll())
    if(!pen_active || e.phase != PenPhase::Down) events.push_back({e, false});
}

const std::vector<CanvasInputEvent>& canvas_input_events() { return events; }

} // namespace mu
