#include <cstring>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

Ref<FramebufferEntt> FramebufferEntt::Create(const char* name) {
  auto f  = cutil::make_ref<FramebufferEntt>();
  f->name = name;
  Project::Get()->entities.push_back(f);
  f->guid_ = Project::Get()->entities.size();
  return f;
}

bool FramebufferEntt::render(Composition* cmp, Image* target, int frame) {
  if(!target || target->empty()) return false;

  if(!captured_) captured_ = cutil::make_ref<Image>();
  if(captured_->width != target->width || captured_->height != target->height) captured_->resize(target->width, target->height);
  std::memcpy(captured_->data(), target->data(), target->size_in_bytes());
  captured_->has_alpha = target->has_alpha;
  captured_->dirty();

  render_filters(cmp, captured_.get(), frame); // trk.filters(色調補正等)をキャプチャ画像に適用してから貼り戻す

  if(clear_original_) target->fill_rgba(Vec4b(0, 0, 0, 0));

  // キャプチャした画像を自身のpos_/scale_/rotation_/alpha_で貼り戻す(Movie::renderと同じ配置ロジック)
  int cw      = target->width;
  int ch      = target->height;
  int base_x  = trk.anchor[0] + (cw - captured_->width) / 2 + pos_[0];
  int base_y  = trk.anchor[1] + (ch - captured_->height) / 2 + pos_[1];
  Vec2 center = Vec2(base_x, base_y) + trk.anchor;
  captured_->copyto(target, center, scale_.avg() / 100, rotation_, alpha_ / 255.0f, trk.blend_);
  return true;
}

} // namespace mu
