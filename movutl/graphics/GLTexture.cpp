#include <GL/glew.h>
#include <memory>
//
#include <movutl/asset/image.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/graphics/GLTexture.hpp>


namespace mu {

GLTexture::GLTexture() {}

inline unsigned int get_opengl_format(ImageFormat t) {
  switch(t) {
    case ImageFormatRGB: return GL_RGB;
    case ImageFormatRGBA: return GL_RGBA;
    case ImageFormatGRAYSCALE: return GL_LUMINANCE;
  }
  return GL_RGB; // Default
}
void GLTexture::set(const std::shared_ptr<Image>& image) {
  MU_ASSERT(image);
  if(!initialized_) {
    glGenTextures(1, &tex_id);
    initialized_ = true;
  }
  img_ = image;

  Image* i = img_.lock().get();

  if(i->width == 0 || i->height == 0) {
    LOG_F(ERROR, "Image size is zero. %s (%d, %d)", i->name, i->width, i->height);
    return;
  }
  auto format = get_opengl_format(i->fmt);
  last_dirty_ = i->dirty_;
  gpu_width_ = i->width;
  gpu_height_ = i->height;

  bind();
  glTexImage2D(GL_TEXTURE_2D, 0, format, i->width, i->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, i->data());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

GLTexture::GLTexture(const std::shared_ptr<Image>& image) {
  set(image);
  bind();
  update_if_necessary();
}

GLTexture::~GLTexture() {
  destroy();
}

bool GLTexture::destroy() {
  if(initialized_) {
    glDeleteTextures(1, &tex_id);
    initialized_ = false;
    return true;
  }
  return false;
}

void GLTexture::bind() const {
  glBindTexture(GL_TEXTURE_2D, tex_id);
}

void GLTexture::unbind() const {
  glBindTexture(GL_TEXTURE_2D, 0);
}

void GLTexture::update_if_necessary() {
  if(img_.expired()) return;
  Image* i = img_.lock().get();
  MU_ASSERT(i);
  if(i->dirty_ == last_dirty_) return;
  bool resized = i->width != gpu_width_ || i->height != gpu_height_;
  if(!initialized_ || resized) {
    destroy();
    set(img_.lock());
  }
  MU_ASSERT(initialized_);
  glBindTexture(GL_TEXTURE_2D, tex_id);
  GLenum format = get_opengl_format(i->fmt);
  glTexImage2D(GL_TEXTURE_2D, 0, format, i->width, i->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, i->data());
  last_dirty_ = i->dirty_;
}

} // namespace mu
