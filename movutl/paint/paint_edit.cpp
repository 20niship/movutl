#include <movutl/paint/paint_document.hpp>
#include <movutl/paint/paint_edit.hpp>

namespace mu {

PaintEdit::PaintEdit(const PaintDocument& d, std::string name) : doc_(d.handle()), name_(std::move(name)) {}

PaintDocument* PaintEdit::target() const {
  auto h = doc_.lock();
  if(!h) return nullptr;
  PaintDocument* d = *h;
  d->cancel_stroke();
  d->mark_changed();
  return d;
}

void StrokeEdit::apply(bool after) {
  auto* d = target();
  if(!d || layer_ >= (int)d->layers.size()) return;
  for(const auto& p : patches_) d->layers[layer_].img.set_tile(p.tx, p.ty, after ? p.after : p.before);
}

} // namespace mu
