#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>

namespace mu {

namespace {
bool from_image(const Ref<Image>& img, Vec2& size) {
  if(!img || img->empty()) return false;
  size = Vec2((float)img->width, (float)img->height);
  return true;
}
} // namespace

bool Movie::source_size(Vec2& size, Vec2& origin_offset) const {
  origin_offset = Vec2(0, 0);
  if(info.width <= 0 || info.height <= 0) return false;
  size = Vec2((float)info.width, (float)info.height);
  return true;
}

bool Image::source_size(Vec2& size, Vec2& origin_offset) const {
  origin_offset = Vec2(0, 0);
  if(width == 0 || height == 0) return false;
  size = Vec2((float)width, (float)height);
  return true;
}

bool ShapeEntt::source_size(Vec2& size, Vec2& origin_offset) const {
  origin_offset = origin_offset_;
  return from_image(img_, size);
}

bool TextEntt::source_size(Vec2& size, Vec2& origin_offset) const {
  origin_offset = Vec2(0, 0);
  return from_image(img_, size);
}

bool FramebufferEntt::source_size(Vec2& size, Vec2& origin_offset) const {
  origin_offset = Vec2(0, 0);
  return from_image(captured_, size);
}

} // namespace mu
