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
  MU_UNUSED(frame);
  MU_UNUSED(cmp);
  if(!target || target->empty()) return false;

  if(!captured_) captured_ = cutil::make_ref<Image>();
  if(captured_->width != target->width || captured_->height != target->height) captured_->resize(target->width, target->height);
  std::memcpy(captured_->data(), target->data(), target->size_in_bytes());
  captured_->has_alpha = target->has_alpha;
  captured_->dirty();

  if(clear_original_) target->fill_rgba(Vec4b(0, 0, 0, 0));
  return true;
}

} // namespace mu
