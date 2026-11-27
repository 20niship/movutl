#include <algorithm>
#include <cmath>
#include <movutl/app/paint_commands.hpp>
#include <movutl/gui/paint/editor_state.hpp>

namespace mu {

PaintEditorState& paint_editor() {
  static PaintEditorState s;
  return s;
}

// ---- 文書 ----
void PaintEditorState::set_document(std::unique_ptr<PaintDocument> d, std::string path) {
  cancel(); // 操作中のツールは古い文書を参照している
  doc_            = std::move(d);
  path_           = std::move(path);
  saved_revision_ = doc_->revision();
  layer_          = (int)doc_->layers.size() - 1;
}

bool PaintEditorState::new_document(int w, int h) {
  if(w <= 0 || h <= 0 || w > kMaxCanvasSize || h > kMaxCanvasSize) return false;
  auto d = std::make_unique<PaintDocument>(w, h);
  d->add_layer("レイヤー 1");
  set_document(std::move(d), "");
  return true;
}

bool PaintEditorState::open(const std::string& path) {
  auto d = PaintDocument::load(path);
  if(!d || d->width() > kMaxCanvasSize || d->height() > kMaxCanvasSize) return false; // 大きすぎると表示用の合成でメモリが尽きる
  set_document(std::move(d), path);
  return true;
}

bool PaintEditorState::save() { return !path_.empty() && save_as(path_); }

bool PaintEditorState::save_as(const std::string& path) {
  if(!doc_ || !doc_->save(path)) return false;
  path_           = path;
  saved_revision_ = doc_->revision();
  return true;
}

// ---- レイヤー ----
int PaintEditorState::active_layer() const { return doc_ ? std::clamp(layer_, 0, std::max(0, (int)doc_->layers.size() - 1)) : -1; }

bool PaintEditorState::commit(std::unique_ptr<PaintEdit> e) {
  if(!e) return false;
  push_paint_edit(std::move(e));
  return true;
}

bool PaintEditorState::add_layer() {
  if(!doc_) return false;
  const int at = std::min(active_layer() + 1, (int)doc_->layers.size());
  if(!commit(doc_->insert_layer(at, "レイヤー " + std::to_string(doc_->layers.size() + 1)))) return false;
  layer_ = at;
  return true;
}

bool PaintEditorState::remove_layer() {
  if(!doc_) return false;
  const int at = active_layer();
  if(!commit(doc_->remove_layer(at))) return false;
  layer_ = std::max(0, at - 1);
  return true;
}

bool PaintEditorState::move_layer(int to) {
  if(!doc_ || !commit(doc_->move_layer(active_layer(), to))) return false;
  layer_ = to;
  return true;
}

bool PaintEditorState::set_layer_props(int i, const LayerProps& p) { return doc_ && commit(doc_->set_layer_props(i, p)); }

// ---- ブラシと色 ----
BrushPreset* PaintEditorState::preset() { return tools.active().uses_brush() ? brushes.selected(tools.active().id) : nullptr; }

BrushSettings PaintEditorState::brush_settings() {
  auto* p         = preset();
  BrushSettings s = p ? p->current : BrushSettings();
  s.color         = color();
  if(color_slot == PaintColorSlot::Transparent) s.engine = "erase";
  return s;
}

bool PaintEditorState::step_brush_size(int dir) {
  auto* p = preset();
  if(!p) return false;
  float& size = p->current.size;
  // 太いほど刻みを大きくする。下げる時は下の区間の刻みを使い、往復で同じ値に戻るようにする
  const float ref  = dir > 0 ? size : size - 0.5f;
  const float step = ref < 10 ? 1 : ref < 30 ? 2 : ref < 100 ? 5 : ref < 200 ? 10 : 50;
  size             = std::clamp(size + (dir > 0 ? step : -step), 1.0f, kMaxBrushSize);
  return true;
}

bool PaintEditorState::step_brush_opacity(int dir) {
  auto* p = preset();
  if(!p) return false;
  // 10%刻みにそろえる
  const float v      = p->current.opacity * 10.0f;
  const float next   = dir > 0 ? std::floor(v + 1.001f) : std::ceil(v - 1.001f);
  p->current.opacity = std::clamp(next / 10.0f, kMinBrushOpacity, 1.0f);
  return true;
}

void PaintEditorState::set_color(const Vec4b& c) {
  if(color_slot == PaintColorSlot::Transparent) color_slot = PaintColorSlot::Main;
  (color_slot == PaintColorSlot::Sub ? sub_color : main_color) = c;
}

void PaintEditorState::swap_color() { color_slot = color_slot == PaintColorSlot::Main ? PaintColorSlot::Sub : PaintColorSlot::Main; }

void PaintEditorState::toggle_transparent() {
  if(color_slot == PaintColorSlot::Transparent) {
    color_slot = prev_slot_;
  } else {
    prev_slot_ = color_slot;
    color_slot = PaintColorSlot::Transparent;
  }
}

// ---- 入力 ----
void PaintEditorState::finish(bool apply) {
  if(apply)
    commit(tools.release());
  else
    tools.cancel();
  if(std::exchange(pen_eraser_, false)) tools.end_temporary();
}

bool PaintEditorState::pointer(const PenEvent& e) {
  switch(e.phase) {
    case PenPhase::Down: {
      if(!doc_) return false;
      if(tools.busy()) finish(true); // Upを取りこぼした
      if(e.eraser) pen_eraser_ = tools.begin_temporary("eraser");
      ToolContext c;
      c.doc           = doc_.get();
      c.layer         = active_layer();
      c.brush         = brush_settings();
      c.on_pick_color = [this](const Vec4b& col) { set_color(col); };
      if(tools.press(c, e.sample)) {
        if(tools.active().uses_brush() && c.brush.engine != "erase") {
          std::erase(color_history, c.brush.color);
          color_history.insert(color_history.begin(), c.brush.color);
          if(color_history.size() > kMaxColorHistory) color_history.pop_back();
        }
        return true;
      }
      finish(false);
      return false;
    }
    case PenPhase::Move: tools.drag(e.sample); return true;
    case PenPhase::Up:
      tools.drag(e.sample);
      finish(true);
      return true;
  }
  return false;
}

} // namespace mu
