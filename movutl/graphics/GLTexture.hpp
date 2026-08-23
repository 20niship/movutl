#include <cutil/ref.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/graphics/GraphicsManager.hpp>

namespace mu {

class GLTexture {
private:
  uint32_t tex_id = 0;
  cutil::WeakPtr<Image> img_;
  uint16_t last_dirty_ = 0;
  int gpu_width_;
  int gpu_height_;
  bool initialized_ = false;

public:
  GLTexture();
  GLTexture(const cutil::Ref<Image>& img);
  void set(const cutil::Ref<Image>& img);
  void bind() const;
  void unbind() const;
  void update_if_necessary();
  bool destroy();
  ~GLTexture();
  operator bool() const { return initialized_; }
  uint32_t get_id() const { return tex_id; }
  bool initialized() const { return initialized_; }
};
} // namespace mu
