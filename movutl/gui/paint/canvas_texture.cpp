#include <GL/glew.h>
// --
#include <movutl/core/profiler.hpp>
#include <movutl/gui/paint/canvas_texture.hpp>

namespace mu {

namespace {
const Vec4b kPaperColor{255, 255, 255, 255};
}

bool CanvasTexture::update(PaintDocument& d) {
  if(d.revision() == revision_) return true;
  MOVUTL_ZONE_SCOPED_N("CanvasTexture::update");
  int max_size = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_size);
  if(d.width() > max_size || d.height() > max_size) return false;

  const bool resize = !tex_ || buf_.width() != d.width() || buf_.height() != d.height();
  const auto rects  = d.composite_dirty(buf_, kPaperColor);
  revision_         = d.revision();
  if(!resize && rects.empty()) return true;

  if(!tex_) {
    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
  glBindTexture(GL_TEXTURE_2D, tex_);
  if(resize) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, d.width(), d.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, buf_.data());
  } else {
    glPixelStorei(GL_UNPACK_ROW_LENGTH, d.width());
    for(const auto& r : rects) glTexSubImage2D(GL_TEXTURE_2D, 0, r.x, r.y, r.w, r.h, GL_RGBA, GL_UNSIGNED_BYTE, &buf_(r.x, r.y));
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  mip_stale_ = true;
  return true;
}

void CanvasTexture::set_sampling(float zoom) {
  if(!tex_) return;
  glBindTexture(GL_TEXTURE_2D, tex_);
  const bool minify = zoom < 1.0f;
  if(minify && mip_stale_) {
    MOVUTL_ZONE_SCOPED_N("CanvasTexture::mipmap");
    glGenerateMipmap(GL_TEXTURE_2D);
    mip_stale_ = false;
  }
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minify ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, zoom >= 2.0f ? GL_NEAREST : GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);
}

} // namespace mu
