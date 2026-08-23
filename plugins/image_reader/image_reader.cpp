#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cstring>
#include <movutl/asset/entity.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <vector>

namespace mu::detail {

// stb_imageはRGBA8を返すが、Image::data_はBGRA8順(video_reader.cppのkOutputPixFmtと同じ)前提
struct StbImageHandle {
  int width = 0, height = 0;
  std::vector<uint8_t> bgra;
};

static bool fn_init() { return true; }
static bool fn_exit() { return true; }

static InputHandle fn_open(const char* file) {
  if(file == nullptr) return nullptr;
  int w = 0, h = 0, ch = 0;
  unsigned char* rgba = stbi_load(file, &w, &h, &ch, 4);
  if(rgba == nullptr) {
    LOG_F(ERROR, "Failed to load image: %s (%s)", file, stbi_failure_reason());
    return nullptr;
  }
  auto handle = new(std::nothrow) StbImageHandle();
  if(handle == nullptr) {
    stbi_image_free(rgba);
    return nullptr;
  }
  handle->width  = w;
  handle->height = h;
  handle->bgra.resize((size_t)w * h * 4);
  for(size_t i = 0; i < (size_t)w * h; i++) {
    handle->bgra[i * 4 + 0] = rgba[i * 4 + 2]; // B
    handle->bgra[i * 4 + 1] = rgba[i * 4 + 1]; // G
    handle->bgra[i * 4 + 2] = rgba[i * 4 + 0]; // R
    handle->bgra[i * 4 + 3] = rgba[i * 4 + 3]; // A
  }
  stbi_image_free(rgba);
  return handle;
}

static bool fn_close(InputHandle ih) {
  delete(StbImageHandle*)ih;
  return true;
}

static bool fn_info_get(InputHandle ih, EntityInfo* iip) {
  if(ih == nullptr || iip == nullptr) return false;
  auto h         = (StbImageHandle*)ih;
  iip->flag      = EntityType_Image;
  iip->rate      = 1;
  iip->scale     = 1;
  iip->framerate = 1;
  iip->nframes   = 1;
  iip->width     = (uint16_t)h->width;
  iip->height    = (uint16_t)h->height;
  iip->format    = ImageFormatRGBA;
  iip->handler   = 0;
  return true;
}

static int fn_read_video(InputHandle ih, int frame, void* buf) {
  if(ih == nullptr || buf == nullptr || frame != 0) return 0; // 静止画はフレーム0のみ
  auto h = (StbImageHandle*)ih;
  std::memcpy(buf, h->bgra.data(), h->bgra.size());
  return (int)h->bgra.size();
}

InputPluginTable plg_image_reader = {
  0x00000002,                                         // guid
  InputPluginFlag_Video | InputPluginFlag_Concurrent, // flag
  EntityType_Image,                                   // supports
  "STB Image Reader",                                 // name
  "",                                                 // filepath
  "Read image files via stb_image",                   // information
  {"png", "jpg", "jpeg", "bmp", "tga", "gif", "hdr", "psd", "", ""},
  fn_init,
  fn_exit,
  fn_open,
  fn_close,
  fn_info_get,
  fn_read_video,
  nullptr, // fn_read_audio
  nullptr, // fn_is_keyframe
  nullptr, // fn_config_wnd
};

} // namespace mu::detail
