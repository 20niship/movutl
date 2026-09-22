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

vec3 rgb2hsvFull(vec3 c) {
  float V = max(c.r, max(c.g, c.b));
  float vmin = min(c.r, min(c.g, c.b));
  float diff = V - vmin;
  float S = (V > 0.0) ? diff / V : 0.0;
  float h = 0.0;
  if(diff > 0.0) {
    float d60 = 60.0 / diff;
    if(V == c.r) h = (c.g - c.b) * d60;
    else if(V == c.g) h = (c.b - c.r) * d60 + 120.0;
    else h = (c.r - c.b) * d60 + 240.0;
  }
  if(h < 0.0) h += 360.0;
  return vec3(h, S, V);
}

vec3 hsv2rgbFull(float h, float s, float v) {
  float cc = v * s;
  float hh = h / 60.0;
  float x = cc * (1.0 - abs(mod(hh, 2.0) - 1.0));
  float m = v - cc;
  vec3 rgb;
  if(hh < 1.0) rgb = vec3(cc, x, 0.0);
  else if(hh < 2.0) rgb = vec3(x, cc, 0.0);
  else if(hh < 3.0) rgb = vec3(0.0, cc, x);
  else if(hh < 4.0) rgb = vec3(0.0, x, cc);
  else if(hh < 5.0) rgb = vec3(x, 0.0, cc);
  else rgb = vec3(cc, 0.0, x);
  return rgb + vec3(m);
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
    // CPU版はcv::Matを介して一度8bitへ量子化してからLUTを引くため、ここでも同じ丸め点を再現する
    vec3 hsv = rgb2hsvFull(rgb);
    float hbyte = mod(floor(hsv.x * 256.0 / 360.0 + 0.5), 256.0);
    float sbyte = clamp(floor(hsv.y * 255.0 + 0.5), 0.0, 255.0);
    float vbyte = clamp(floor(hsv.z * 255.0 + 0.5), 0.0, 255.0);
    float hue_off = floor(mod(pc.hue + 360.0, 360.0) * 256.0 / 360.0 + 0.5);
    float h2 = mod(hbyte + hue_off, 256.0);
    float s2 = clamp(floor(sbyte * pc.saturation / 100.0 + 0.5), 0.0, 255.0);
    rgb = hsv2rgbFull(h2 * 360.0 / 256.0, s2 / 255.0, vbyte / 255.0);
    rgb = floor(clamp(rgb * 255.0, 0.0, 255.0) + 0.5) / 255.0; // HSV2RGB_FULLも8bit量子化された行列を返す
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
