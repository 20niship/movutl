#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <movutl/asset/image.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/output.hpp>
#include <movutl/plugin/plugin.hpp>
#include <vector>

namespace mu::detail {

namespace fs = std::filesystem;

struct PngExportHandle {
  fs::path dir;
  std::string stem;
  int digits = 6;
};

static bool fn_init(cutil::PropInfo* props, cutil::Prop* defaults) {
  props->fields.push_back(cutil::PropInfo::Field("digits", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("連番桁数");
  props->fields.back().min_value = 1.0f;
  props->fields.back().max_value = 10.0f;
  defaults->set<int32_t>("digits", 6);
  return true;
}

static bool fn_exit() { return true; }

static void* fn_open(const char* path, int, int, float, const cutil::Prop& props) {
  if(path == nullptr) return nullptr;
  auto h = new(std::nothrow) PngExportHandle();
  if(h == nullptr) return nullptr;
  fs::path p = path;
  h->dir     = p.has_parent_path() ? p.parent_path() : fs::path(".");
  h->stem    = p.stem().string();
  h->digits  = cutil::get_or<int32_t>(props, "digits", 6);
  return h;
}

// Image::data_はBGRA8順のため、stb_image_writeへ渡す前にRGBA8へ並び替える(image_readerの逆変換)
static bool fn_write_frame(void* handle, const Image* img, int frame) {
  if(handle == nullptr || img == nullptr) return false;
  auto h = (PngExportHandle*)handle;

  char numbuf[16];
  std::snprintf(numbuf, sizeof(numbuf), "%0*d", h->digits, frame);
  fs::path out = h->dir / (h->stem + "_" + numbuf + ".png");

  std::vector<uint8_t> rgba(img->size() * 4);
  for(size_t i = 0; i < img->size(); i++) {
    const Vec4b& px = (*img)[i];
    rgba[i * 4 + 0] = px[2]; // R
    rgba[i * 4 + 1] = px[1]; // G
    rgba[i * 4 + 2] = px[0]; // B
    rgba[i * 4 + 3] = px[3]; // A
  }

  if(stbi_write_png(out.string().c_str(), (int)img->width, (int)img->height, 4, rgba.data(), (int)img->width * 4) == 0) {
    LOG_F(ERROR, "Failed to write png: %s", out.string().c_str());
    return false;
  }
  return true;
}

static bool fn_close(void* handle) {
  delete(PngExportHandle*)handle;
  return true;
}

OutputPluginTable out_export_png = {
  0x00000101, // guid
  "PNG Image/Sequence",
  "stb_image_writeを用いたpng連番/静止画書き出しプラグイン",
  true, // is_sequence
  {"png", "", "", "", "", "", "", "", "", ""},
  fn_init,
  fn_exit,
  fn_open,
  fn_write_frame,
  fn_close,
};

} // namespace mu::detail

namespace {

void plugin_init(mu::ABIContext* abi) { abi->register_output_plugin(&mu::detail::out_export_png); }
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "PNG Exporter", sizeof(table->name) - 1);
  std::strncpy(table->description, "stb_image_writeを用いたpng書き出しプラグイン", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
