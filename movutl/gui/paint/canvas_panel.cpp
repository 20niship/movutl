#include <imgui.h>
// --
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <movutl/gui/paint/canvas_input.hpp>
#include <movutl/gui/paint/canvas_panel.hpp>
#include <movutl/gui/paint/canvas_texture.hpp>
#include <movutl/gui/paint/editor_state.hpp>
#include <movutl/gui/workspace_presets.hpp>
#include <optional>

namespace mu {

namespace {

constexpr int kDefaultWidth = 1920, kDefaultHeight = 1080;
constexpr float kPi            = 3.14159265f;
constexpr float kClickSlop     = 4.0f;  // これ以下の移動はクリック扱い(screen px)
constexpr float kZoomPerPixel  = 0.01f; // 拡大縮小ドラッグの速さ
constexpr float kZoomPerNotch  = 1.15f; // ホイール1段の倍率
constexpr float kZoomClickStep = 2.0f;

enum class Mode { Draw, Pan, Rotate, ZoomIn, ZoomOut, Size };

// 今押されているキーで、次の左ドラッグが何になるか
Mode key_mode() {
  if(is_shortcut_held("space")) return Mode::Pan;
  if(is_shortcut_held("shift+space") || is_shortcut_held("r")) return Mode::Rotate;
  if(is_shortcut_held("ctrl+space")) return Mode::ZoomIn;
  if(is_shortcut_held("alt+space")) return Mode::ZoomOut;
  if(is_shortcut_held("ctrl+alt")) return Mode::Size;
  return Mode::Draw;
}

Vec2 vec(const ImVec2& v) { return Vec2(v.x, v.y); }
ImVec2 im(const Vec2& v) { return ImVec2(v[0], v[1]); }

class CanvasPanel final : public UIPanel {
public:
  void Update() override;

private:
  struct Drag {
    Mode mode;
    bool pen;
    Vec2 start;    // 開始位置(screen)
    PenEvent last; // 直近の入力(screen)
    float start_size = 0;
    bool moved       = false;
  };

  void handle_input(bool hovered);
  void begin_drag(const CanvasInputEvent& in);
  void drag_to(PenEvent e);
  void end_drag(); // Upが届かないまま操作を終える
  void fit();      // 全体表示(回転と反転は保つ)
  void fit_if_needed();
  void draw_canvas(bool hovered);
  void draw_cursor(bool hovered);
  void draw_bar();
  Vec2 view_center() const;

  CanvasTexture tex_;
  std::optional<Drag> drag_;
  std::weak_ptr<PaintDocument*> seen_; // 表示中の文書(切り替わったら全体表示にする)
  Vec2 fit_view_size_;
  float fit_zoom_ = -1;
  bool alt_pick_  = false; // Altで一時的にスポイトにしている
};

Vec2 CanvasPanel::view_center() const {
  const auto& v = paint_editor().view;
  return Vec2(v.view_pos[0] + v.view_size[0] * 0.5f, v.view_pos[1] + v.view_size[1] * 0.5f);
}

void CanvasPanel::fit() {
  auto& v = paint_editor().view;
  v.fit();
  v.zoom         = std::max(CanvasView::kMinZoom, v.zoom * 0.95f); // 周りに少し余白を残す
  fit_zoom_      = v.zoom;
  fit_view_size_ = v.view_size;
}

void CanvasPanel::fit_if_needed() {
  auto& st               = paint_editor();
  auto& v                = st.view;
  const auto old         = seen_.lock();
  const bool changed_doc = !old || *old != st.doc();
  // 全体表示のまま操作していなければ、ウィンドウの大きさに追従する
  const bool resized = v.view_size != fit_view_size_ && v.zoom == fit_zoom_ && v.pan == Vec2(0, 0);
  if(!changed_doc && !resized) return;
  if(changed_doc) {
    seen_   = st.doc()->handle();
    v.angle = 0;
    v.flip  = false;
  }
  fit();
}

void CanvasPanel::begin_drag(const CanvasInputEvent& in) {
  auto& st = paint_editor();
  Drag d{key_mode(), in.pen, Vec2(in.event.sample.x, in.event.sample.y), in.event};
  if(d.mode == Mode::Size) {
    auto* p = st.preset();
    if(!p) return;
    d.start_size = p->current.size;
  }
  if(d.mode == Mode::Draw) {
    PenEvent e   = in.event;
    const Vec2 c = st.view.to_canvas(d.start);
    e.sample.x   = c[0];
    e.sample.y   = c[1];
    if(!st.pointer(e)) return;
  }
  drag_ = d;
}

void CanvasPanel::drag_to(PenEvent e) {
  auto& st = paint_editor();
  auto& v  = st.view;
  Drag& d  = *drag_;
  const Vec2 p(e.sample.x, e.sample.y), last(d.last.sample.x, d.last.sample.y);
  const bool up = e.phase == PenPhase::Up;
  d.last        = e;
  switch(d.mode) {
    case Mode::Draw: {
      const Vec2 c = v.to_canvas(p);
      e.sample.x   = c[0];
      e.sample.y   = c[1];
      st.pointer(e);
      break;
    }
    case Mode::Pan: v.move(Vec2(p[0] - last[0], p[1] - last[1])); break;
    case Mode::Rotate: {
      const Vec2 c  = view_center();
      const float a = std::atan2(p[1] - c[1], p[0] - c[0]) - std::atan2(last[1] - c[1], last[0] - c[0]);
      v.rotate_at(c, std::remainder(a, 2 * kPi));
      break;
    }
    case Mode::ZoomIn:
    case Mode::ZoomOut:
      d.moved = d.moved || std::hypot(p[0] - d.start[0], p[1] - d.start[1]) > kClickSlop;
      if(d.moved)
        v.zoom_at(d.start, std::exp((p[0] - last[0]) * kZoomPerPixel));
      else if(up)
        v.zoom_at(d.start, d.mode == Mode::ZoomIn ? kZoomClickStep : 1.0f / kZoomClickStep);
      break;
    case Mode::Size:
      // カーソルの円の半径がドラッグ量に追従する
      if(auto* preset = st.preset()) preset->current.size = std::clamp(d.start_size + (p[0] - d.start[0]) * 2.0f / v.zoom, 1.0f, PaintEditorState::kMaxBrushSize);
      break;
  }
  if(up) drag_.reset();
}

void CanvasPanel::end_drag() {
  if(!drag_) return;
  PenEvent e = drag_->last;
  e.phase    = PenPhase::Up;
  drag_to(e);
}

void CanvasPanel::handle_input(bool hovered) {
  auto& st = paint_editor();
  auto& v  = st.view;
  auto& io = ImGui::GetIO();

  // Altを押している間はスポイト(操作の開始より先に切り替える)
  if(!st.tools.busy() && !drag_) {
    if(is_shortcut_held("alt") && !ImGui::IsKeyDown(ImGuiKey_Space) && st.tools.selected().uses_brush())
      alt_pick_ = st.tools.begin_temporary("eyedropper");
    else if(std::exchange(alt_pick_, false))
      st.tools.end_temporary();
  }

  for(const auto& in : canvas_input_events()) {
    if(in.event.phase == PenPhase::Down) {
      const float x = in.event.sample.x - v.view_pos[0], y = in.event.sample.y - v.view_pos[1];
      if(!hovered || x < 0 || y < 0 || x >= v.view_size[0] || y >= v.view_size[1]) continue;
      if(drag_) {
        if(drag_->pen == in.pen) {
          end_drag(); // Upを取りこぼした
        } else if(in.pen) {
          st.cancel(); // 同じ接触がマウスとして先に届いていた
          drag_.reset();
        } else {
          continue;
        }
      }
      begin_drag(in);
    } else if(drag_ && drag_->pen == in.pen) {
      drag_to(in.event);
    }
  }

  if(drag_ && drag_->mode == Mode::Draw && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
    st.cancel();
    drag_.reset();
  }
  if(ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Middle)) v.move(vec(io.MouseDelta));
  if(hovered && io.MouseWheel != 0.0f) v.zoom_at(vec(io.MousePos), std::pow(kZoomPerNotch, io.MouseWheel));
  st.tick(io.DeltaTime);
}

void CanvasPanel::draw_cursor(bool hovered) {
  auto& st        = paint_editor();
  const bool size = drag_ && drag_->mode == Mode::Size;
  if(ImGui::IsItemActive() && ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    return;
  }
  if(!hovered && !(drag_ && (drag_->mode == Mode::Draw || size))) return;
  switch(drag_ ? drag_->mode : key_mode()) {
    case Mode::Pan: ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); break;
    case Mode::Rotate: ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll); break;
    case Mode::ZoomIn:
    case Mode::ZoomOut: ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW); break;
    case Mode::Draw:
    case Mode::Size: {
      const auto* p = st.preset();
      if(!p) break;
      const float r  = p->current.size * 0.5f * st.view.zoom;
      const ImVec2 c = size ? im(drag_->start) : ImGui::GetIO().MousePos;
      auto* dl       = ImGui::GetWindowDrawList();
      // 明るい所でも暗い所でも見えるように黒と白を重ねる
      dl->AddCircle(c, r, IM_COL32(0, 0, 0, 200), 0, 2.5f);
      dl->AddCircle(c, r, IM_COL32(255, 255, 255, 230), 0, 1.0f);
      if(r >= 3.0f) {
        dl->AddCircleFilled(c, 1.5f, IM_COL32(0, 0, 0, 200));
        ImGui::SetMouseCursor(ImGuiMouseCursor_None);
      }
      break;
    }
  }
}

void CanvasPanel::draw_canvas(bool hovered) {
  auto& st       = paint_editor();
  const auto& v  = st.view;
  auto* dl       = ImGui::GetWindowDrawList();
  const ImVec2 a = im(v.view_pos), b(a.x + v.view_size[0], a.y + v.view_size[1]);
  dl->PushClipRect(a, b, true);
  dl->AddRectFilled(a, b, IM_COL32(60, 60, 60, 255));
  if(tex_.update(*st.doc())) {
    tex_.set_sampling(v.zoom);
    const float w = v.canvas_size[0], h = v.canvas_size[1];
    const ImVec2 q[4] = {im(v.to_screen(Vec2(0, 0))), im(v.to_screen(Vec2(w, 0))), im(v.to_screen(Vec2(w, h))), im(v.to_screen(Vec2(0, h)))};
    dl->AddImageQuad(tex_.id(), q[0], q[1], q[2], q[3], ImVec2(0, 0), ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1));
    dl->AddQuad(q[0], q[1], q[2], q[3], IM_COL32(0, 0, 0, 255));
  } else {
    dl->AddText(ImVec2(a.x + 8, a.y + 8), IM_COL32(255, 120, 120, 255), "キャンバスが大きすぎて表示できません");
  }
  draw_cursor(hovered);
  dl->PopClipRect();
}

void CanvasPanel::draw_bar() {
  auto& st = paint_editor();
  auto& v  = st.view;
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("ズーム");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(130);
  float percent = v.zoom * 100.0f;
  if(ImGui::SliderFloat("##zoom", &percent, CanvasView::kMinZoom * 100, CanvasView::kMaxZoom * 100, "%.1f%%", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp)) v.zoom_at(view_center(), percent / 100.0f / v.zoom);
  ImGui::SameLine();
  if(ImGui::Button("100%")) v.zoom_at(view_center(), 1.0f / v.zoom);
  ImGui::SameLine();
  if(ImGui::Button("全体表示")) fit();
  ImGui::SameLine(0, 16);
  ImGui::TextUnformatted("回転");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(130);
  float degree = v.angle * 180.0f / kPi;
  if(ImGui::SliderFloat("##angle", &degree, -180.0f, 180.0f, "%.0f°", ImGuiSliderFlags_AlwaysClamp)) v.rotate_at(view_center(), degree * kPi / 180.0f - v.angle);
  ImGui::SameLine();
  if(ImGui::Button("0°")) v.rotate_at(view_center(), -v.angle);
  ImGui::SameLine(0, 16);
  bool flip = v.flip;
  if(ImGui::Checkbox("左右反転", &flip)) v.flip_horizontal();
  ImGui::SameLine(0, 16);
  const std::string name = st.path().empty() ? "(無題)" : std::filesystem::path(st.path()).filename().string();
  ImGui::Text("%s%s  %d x %d", name.c_str(), st.modified() ? " *" : "", st.doc()->width(), st.doc()->height());
}

void CanvasPanel::Update() {
  auto& st = paint_editor();
  if(!st.doc()) st.new_document(kDefaultWidth, kDefaultHeight);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const bool visible = ImGui::Begin(paint_window_title("canvas").c_str(), &open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleVar();
  if(!visible) {
    end_drag();
    ImGui::End();
    return;
  }
  if(ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) || ImGui::IsWindowHovered()) set_shortcut_context("canvas");

  const ImVec2 pos = ImGui::GetCursorScreenPos(), avail = ImGui::GetContentRegionAvail();
  const float bar_height = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y * 2;
  st.view.view_pos       = vec(pos);
  st.view.view_size      = Vec2(std::max(avail.x, 1.0f), std::max(avail.y - bar_height, 1.0f));
  st.view.canvas_size    = Vec2((float)st.doc()->width(), (float)st.doc()->height());
  fit_if_needed();

  ImGui::InvisibleButton("##canvas_view", im(st.view.view_size), ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle);
  const bool hovered = ImGui::IsItemHovered();
  handle_input(hovered);
  draw_canvas(hovered);

  ImGui::SetCursorScreenPos(ImVec2(pos.x + 8, pos.y + st.view.view_size[1] + ImGui::GetStyle().ItemSpacing.y));
  draw_bar();
  ImGui::End();
}

} // namespace

Ref<UIPanel> make_canvas_panel() { return cutil::make_ref<CanvasPanel>(); }

} // namespace mu
