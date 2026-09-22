#include <cstring>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/gpu/gpu_effects.hpp>
#include <movutl/vulkan/gpu_compute.hpp>

namespace mu::detail {

namespace {
// f_invert(image_tone_filter.cpp)と同じ: RGBを反転(rgba8はimageLoad/Storeで0..1に正規化されるため1.0-cで良い)
constexpr const char* kInvertGlsl = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform writeonly image2D dstImg;
layout(push_constant) uniform PC { int invert_alpha; } pc;
void main() {
  ivec2 sz = imageSize(dstImg);
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  vec4 c = imageLoad(srcImg, p);
  c.rgb = vec3(1.0) - c.rgb;
  if(pc.invert_alpha != 0) c.a = 1.0 - c.a;
  imageStore(dstImg, p, c);
}
)GLSL";

// f_color_correction(image_color_filter.cpp)と同じ: hue/saturation指定時はHSV_FULL相当(OpenCVと同じ8bit量子化点)で色相/彩度を変えた上で明度/コントラストLUTを適用する
constexpr const char* kColorCorrectionGlsl = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform writeonly image2D dstImg;
layout(push_constant) uniform PC {
  float brightness, contrast, hue, saturation;
} pc;

// OpenCVのCOLOR_RGB2HSV_FULL(8u)と同じ整数シフトテーブル式(hsv_shift=12)。H/S/Vは0..255のバイト値で返す
ivec3 rgb2hsvFullByte(ivec3 c) {
  int r = c.r, g = c.g, b = c.b;
  int v = max(r, max(g, b));
  int vmin = min(r, min(g, b));
  int diff = v - vmin;
  int vr = (v == r) ? -1 : 0;
  int vg = (v == g) ? -1 : 0;
  int h_raw = (vr & (g - b)) + (~vr & ((vg & (b - r + 2 * diff)) + (~vg & (r - g + 4 * diff))));
  int sdiv = (v == 0) ? 0 : int(floor(1044480.0 / float(v) + 0.5));     // (255<<12)/v
  int hdiv = (diff == 0) ? 0 : int(floor(1048576.0 / (6.0 * float(diff)) + 0.5)); // (256<<12)/(6*diff)
  int s = (diff * sdiv + 2048) >> 12;
  int h = (h_raw * hdiv + 2048) >> 12;
  if(h < 0) h += 256;
  return ivec3(h & 255, clamp(s, 0, 255), v);
}

// OpenCVのCOLOR_HSV2RGB_FULL(8u→float経路)と同じsector展開式。h/s/vは0..255のバイト値、戻り値は0..1
vec3 hsv2rgbFullByte(int hb, int sb, int vb) {
  float h6 = float(hb) * (6.0 / 256.0);
  float s = float(sb) / 255.0, v = float(vb) / 255.0;
  if(h6 < 0.0) h6 += 6.0;
  else if(h6 >= 6.0) h6 -= 6.0;
  int sector = int(floor(h6));
  float frac = h6 - float(sector);
  float tab0 = v, tab1 = v * (1.0 - s), tab2 = v * (1.0 - s * frac), tab3 = v * (1.0 - s * (1.0 - frac));
  // OpenCVのsector_data[sector]={b,g,rのtab[]インデックス}をr,g,bの順に並べ替えたもの
  if(sector == 0) return vec3(tab0, tab3, tab1);
  if(sector == 1) return vec3(tab2, tab0, tab1);
  if(sector == 2) return vec3(tab1, tab0, tab3);
  if(sector == 3) return vec3(tab1, tab2, tab0);
  if(sector == 4) return vec3(tab3, tab1, tab0);
  return vec3(tab0, tab1, tab2);
}

float lut_bc(float v255) {
  float f = (v255 - 127.0) * pc.contrast + 127.0;
  f *= pc.brightness;
  return clamp(f, 0.0, 255.0);
}

void main() {
  ivec2 sz = imageSize(dstImg);
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  vec4 c = imageLoad(srcImg, p);
  vec3 rgb = c.rgb;

  if(pc.hue != 0.0 || pc.saturation != 100.0) {
    // CPU版(cv::cvtColor COLOR_RGB2HSV_FULL/HSV2RGB_FULL)と同じ整数シフトテーブル式でH/S/Vバイトを再現する
    ivec3 c255 = ivec3(round(rgb * 255.0));
    ivec3 hsv = rgb2hsvFullByte(c255);
    int hue_off = int(floor(mod(pc.hue + 360.0, 360.0) * 256.0 / 360.0 + 0.5));
    int h2 = (hsv.x + hue_off) & 255;
    int s2 = clamp(int(floor(float(hsv.y) * pc.saturation / 100.0 + 0.5)), 0, 255);
    rgb = hsv2rgbFullByte(h2, s2, hsv.z);
  }

  vec3 v255 = rgb * 255.0;
  vec3 outc = vec3(lut_bc(v255.r), lut_bc(v255.g), lut_bc(v255.b)) / 255.0;
  imageStore(dstImg, p, vec4(outc, c.a));
}
)GLSL";

// image_tile_filter.cppのfn_proc_tileと同じ整数式(sx=(x*nx)%w)。出力=元画像と同じサイズであることが前提
constexpr const char* kTileGlsl = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform writeonly image2D dstImg;
layout(push_constant) uniform PC { int nx, ny, src_w, src_h; } pc;
void main() {
  ivec2 sz = imageSize(dstImg);
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  int sx = (p.x * pc.nx) % sz.x;
  int sy = (p.y * pc.ny) % sz.y;
  imageStore(dstImg, p, imageLoad(srcImg, ivec2(sx, sy)));
}
)GLSL";

bool run_inplace(const char* glsl, Image& img, const void* push, uint32_t push_size) {
  Image out(img.width, img.height);
  ComputeParams params;
  params.push      = push;
  params.push_size = push_size;
  std::string err;
  if(!run_compute_image(glsl, {&img}, out, params, &err)) {
    LOG_F(ERROR, "gpu_effects: %s", err.c_str());
    return false;
  }
  std::memcpy(img.data(), out.data(), img.size_in_bytes()); // Entity(mtx)がコピー不可のためImage全体代入はできず、画素データのみコピーする
  img.dirty();
  return true;
}
} // namespace

bool gpu_invert(Image& img, bool invert_alpha) {
  struct {
    int32_t invert_alpha;
  } pc{invert_alpha ? 1 : 0};
  return run_inplace(kInvertGlsl, img, &pc, sizeof(pc));
}

bool gpu_color_correction(Image& img, float brightness, float contrast, float hue, float saturation) {
  struct {
    float brightness, contrast, hue, saturation;
  } pc{brightness / 100.0f, contrast / 100.0f, hue, saturation};
  return run_inplace(kColorCorrectionGlsl, img, &pc, sizeof(pc));
}

bool gpu_tile(Image& img, int nx, int ny) {
  if(nx <= 0 || ny <= 0) return false;
  struct {
    int32_t nx, ny, src_w, src_h;
  } pc{nx, ny, (int32_t)img.width, (int32_t)img.height};
  return run_inplace(kTileGlsl, img, &pc, sizeof(pc));
}

} // namespace mu::detail
