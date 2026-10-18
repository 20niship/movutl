#include <algorithm>
#include <cmath>
#include <cstring>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/text_encoding.hpp>
#include <movutl/plugin/gpu/filter2_compat.hpp>
#include <movutl/vulkan/gpu_compute.hpp>
#include <movutl/vulkan/vk_image.hpp>
#include <mutex>

namespace mu::detail {

std::string wstr_to_utf8(const wchar_t* s) {
  if(!s) return {};
  std::string out;
  for(const wchar_t* p = s; *p; p++) {
    uint32_t cp = (uint32_t)*p;
    if(cp < 0x80) {
      out.push_back((char)cp);
    } else if(cp < 0x800) {
      out.push_back((char)(0xC0 | (cp >> 6)));
      out.push_back((char)(0x80 | (cp & 0x3F)));
    } else if(cp < 0x10000) {
      out.push_back((char)(0xE0 | (cp >> 12)));
      out.push_back((char)(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back((char)(0x80 | (cp & 0x3F)));
    } else {
      out.push_back((char)(0xF0 | (cp >> 18)));
      out.push_back((char)(0x80 | ((cp >> 12) & 0x3F)));
      out.push_back((char)(0x80 | ((cp >> 6) & 0x3F)));
      out.push_back((char)(0x80 | (cp & 0x3F)));
    }
  }
  return out;
}

namespace {
// cache:xxxx/image:xxxx/randomはfilter2.hの実際の意味論(レンダリング処理共用・VRAMキャッシュ)に合わせプロセス内で共有する
std::mutex g_shared_mtx;
std::unordered_map<std::string, std::unique_ptr<GpuImage>> g_cache_resources; // "cache:xxxx"
std::unordered_map<std::string, std::unique_ptr<GpuImage>> g_image_resources; // "image:xxxx"(パスがキー)
std::unique_ptr<GpuImage> g_random;

bool is_glsl_shader_path(const std::string& ext) { return ext == "frag" || ext == "comp" || ext == "glsl"; }

bool load_glsl_from_file(const std::wstring& path_w, std::string& out) {
  std::string path = wstr_to_utf8(path_w.c_str());
  std::string ext  = fs_extension(path);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  if(ext == "cso" || ext == "spv") {
    LOG_F(ERROR, "filter2_compat: %s は非対応です(.cso/.spvはGLSL文字列ではないため実行できません)", path.c_str());
    return false;
  }
  if(!is_glsl_shader_path(ext)) {
    LOG_F(ERROR, "filter2_compat: 未知のシェーダ拡張子です: %s", path.c_str());
    return false;
  }
  return read_text_file_utf8(path, out);
}

// VERTEX_COLOR等の頂点3つ(三角形1枚)を扱う際の座標変換(dst中心原点・Y下向き→dst左上原点)
struct ScreenVert {
  float x, y;
};
ScreenVert to_screen(float vx, float vy, int w, int h) { return {vx + w / 2.0f, vy + h / 2.0f}; }

constexpr const char* kBlendFn = R"GLSL(
int div255(int x) { return (x + 128 + ((x + 128) >> 8)) >> 8; }
int blend_channel(int mode, int d, int s) {
  if(mode == 1) return min(255, d + s);
  if(mode == 2) return max(0, d - s);
  if(mode == 3) return div255(d * s);
  if(mode == 4) return d + s - d * s / 255;
  if(mode == 5) return d < 128 ? 2 * d * s / 255 : 255 - 2 * (255 - d) * (255 - s) / 255;
  if(mode == 6) return min(d, s);
  if(mode == 7) return max(d, s);
  return s;
}
)GLSL";

// draw_image: 2D簡略化(z/rx/ry/szは無視)のtranslate+rotate+scale+alpha合成。ponytail: vulkan_renderer.cppのkCompositeGlsl(Phase4で検証済み)と似るが、壊さないよう複製する(共通化は将来の課題)
constexpr const char* kDrawImageGlslTemplate = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform image2D dstImg;
layout(push_constant) uniform PC {
  float cx, cy, cos_r, sin_r, sx, sy;
  int bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y;
  int src_w, src_h, am256, blend_mode;
} pc;
%s
void main() {
  ivec2 dp = ivec2(gl_GlobalInvocationID.xy) + ivec2(pc.bbox_min_x, pc.bbox_min_y);
  if(dp.x >= pc.bbox_max_x || dp.y >= pc.bbox_max_y) return;
  float dx = float(dp.x) - pc.cx, dy = float(dp.y) - pc.cy;
  float lx = (dx * pc.cos_r + dy * pc.sin_r) / pc.sx;
  float ly = (-dx * pc.sin_r + dy * pc.cos_r) / pc.sy;
  int sx = int(floor(lx + pc.src_w / 2.0)), sy = int(floor(ly + pc.src_h / 2.0));
  if(sx < 0 || sx >= pc.src_w || sy < 0 || sy >= pc.src_h) return;
  ivec4 s255 = ivec4(round(imageLoad(srcImg, ivec2(sx, sy)) * 255.0));
  int a = ((s255.a + (s255.a >> 7)) * pc.am256) >> 8;
  if(a <= 0) return;
  ivec4 d255 = ivec4(round(imageLoad(dstImg, dp) * 255.0));
  ivec4 outc = d255;
  for(int c = 0; c < 3; c++) {
    int bl = (pc.blend_mode == 0) ? s255[c] : blend_channel(pc.blend_mode, d255[c], s255[c]);
    outc[c] = d255[c] + (((bl - d255[c]) * a) >> 8);
  }
  outc.a = d255.a + (((255 - d255.a) * a) >> 8);
  imageStore(dstImg, dp, vec4(outc) / 255.0);
}
)GLSL";

std::string draw_image_glsl() {
  static std::string s = [] {
    std::string t = kDrawImageGlslTemplate;
    size_t pos    = t.find("%s");
    t.replace(pos, 2, kBlendFn);
    return t;
  }();
  return s;
}

struct DrawImagePC {
  float cx, cy, cos_r, sin_r, sx, sy;
  int32_t bbox_min[2], bbox_max[2];
  int32_t src_w, src_h, am256, blend_mode;
};

// 三角形ラスタライザ(バリセントリック座標判定)。頂点数は少数想定のため三角形ごとにdispatchする(ponytail: 大量ポリゴンでは非効率、必要になればSSBOで一括化する)
constexpr const char* kPolyColorGlslTemplate = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform image2D dstImg;
layout(push_constant) uniform PC {
  float x0,y0,x1,y1,x2,y2;
  float r0,g0,b0,a0, r1,g1,b1,a1, r2,g2,b2,a2;
  int bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, blend_mode;
} pc;
%s
void main() {
  ivec2 dp = ivec2(gl_GlobalInvocationID.xy) + ivec2(pc.bbox_min_x, pc.bbox_min_y);
  if(dp.x >= pc.bbox_max_x || dp.y >= pc.bbox_max_y) return;
  float px = float(dp.x) + 0.5, py = float(dp.y) + 0.5;
  float d = (pc.y1-pc.y2)*(pc.x0-pc.x2) + (pc.x2-pc.x1)*(pc.y0-pc.y2);
  if(abs(d) < 1e-6) return;
  float w0 = ((pc.y1-pc.y2)*(px-pc.x2) + (pc.x2-pc.x1)*(py-pc.y2)) / d;
  float w1 = ((pc.y2-pc.y0)*(px-pc.x2) + (pc.x0-pc.x2)*(py-pc.y2)) / d;
  float w2 = 1.0 - w0 - w1;
  if(w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4) return;
  vec4 srcc = vec4(w0*pc.r0+w1*pc.r1+w2*pc.r2, w0*pc.g0+w1*pc.g1+w2*pc.g2, w0*pc.b0+w1*pc.b1+w2*pc.b2, w0*pc.a0+w1*pc.a1+w2*pc.a2);
  ivec4 s255 = ivec4(round(clamp(srcc, 0.0, 1.0) * 255.0));
  int a = s255.a;
  if(a <= 0) return;
  ivec4 d255 = ivec4(round(imageLoad(dstImg, dp) * 255.0));
  ivec4 outc = d255;
  for(int c = 0; c < 3; c++) {
    int bl = (pc.blend_mode == 0) ? s255[c] : blend_channel(pc.blend_mode, d255[c], s255[c]);
    outc[c] = d255[c] + (((bl - d255[c]) * (a + (a >> 7))) >> 8);
  }
  outc.a = d255.a + (((255 - d255.a) * (a + (a >> 7))) >> 8);
  imageStore(dstImg, dp, vec4(outc) / 255.0);
}
)GLSL";

constexpr const char* kPolyTextureGlslTemplate = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform image2D dstImg;
layout(push_constant) uniform PC {
  float x0,y0,x1,y1,x2,y2;
  float u0,v0,a0, u1,v1,a1, u2,v2,a2;
  int bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, src_w, src_h, blend_mode;
} pc;
%s
void main() {
  ivec2 dp = ivec2(gl_GlobalInvocationID.xy) + ivec2(pc.bbox_min_x, pc.bbox_min_y);
  if(dp.x >= pc.bbox_max_x || dp.y >= pc.bbox_max_y) return;
  float px = float(dp.x) + 0.5, py = float(dp.y) + 0.5;
  float d = (pc.y1-pc.y2)*(pc.x0-pc.x2) + (pc.x2-pc.x1)*(pc.y0-pc.y2);
  if(abs(d) < 1e-6) return;
  float w0 = ((pc.y1-pc.y2)*(px-pc.x2) + (pc.x2-pc.x1)*(py-pc.y2)) / d;
  float w1 = ((pc.y2-pc.y0)*(px-pc.x2) + (pc.x0-pc.x2)*(py-pc.y2)) / d;
  float w2 = 1.0 - w0 - w1;
  if(w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4) return;
  float u = w0*pc.u0+w1*pc.u1+w2*pc.u2, v = w0*pc.v0+w1*pc.v1+w2*pc.v2, av = w0*pc.a0+w1*pc.a1+w2*pc.a2;
  int sx = clamp(int(u * float(pc.src_w)), 0, pc.src_w - 1), sy = clamp(int(v * float(pc.src_h)), 0, pc.src_h - 1);
  ivec4 s255 = ivec4(round(imageLoad(srcImg, ivec2(sx, sy)) * 255.0));
  int a = int(clamp(av, 0.0, 1.0) * float(s255.a));
  if(a <= 0) return;
  ivec4 d255 = ivec4(round(imageLoad(dstImg, dp) * 255.0));
  ivec4 outc = d255;
  for(int c = 0; c < 3; c++) {
    int bl = (pc.blend_mode == 0) ? s255[c] : blend_channel(pc.blend_mode, d255[c], s255[c]);
    outc[c] = d255[c] + (((bl - d255[c]) * (a + (a >> 7))) >> 8);
  }
  outc.a = d255.a + (((255 - d255.a) * (a + (a >> 7))) >> 8);
  imageStore(dstImg, dp, vec4(outc) / 255.0);
}
)GLSL";

std::string poly_color_glsl() {
  static std::string s = [] {
    std::string t = kPolyColorGlslTemplate;
    t.replace(t.find("%s"), 2, kBlendFn);
    return t;
  }();
  return s;
}
std::string poly_texture_glsl() {
  static std::string s = [] {
    std::string t = kPolyTextureGlslTemplate;
    t.replace(t.find("%s"), 2, kBlendFn);
    return t;
  }();
  return s;
}

struct PolyColorPC {
  float xy[6];
  float rgba[12];
  int32_t bbox_min[2], bbox_max[2], blend_mode;
};
struct PolyTexturePC {
  float xy[6];
  float uva[9];
  int32_t bbox_min[2], bbox_max[2], src_w, src_h, blend_mode;
};

bool bbox_from_tri(const float xy[6], int w, int h, int32_t out_min[2], int32_t out_max[2]) {
  float minx = std::min({xy[0], xy[2], xy[4]}), maxx = std::max({xy[0], xy[2], xy[4]});
  float miny = std::min({xy[1], xy[3], xy[5]}), maxy = std::max({xy[1], xy[3], xy[5]});
  out_min[0] = std::max(0, (int)std::floor(minx)), out_min[1] = std::max(0, (int)std::floor(miny));
  out_max[0] = std::min(w, (int)std::ceil(maxx)), out_max[1] = std::min(h, (int)std::ceil(maxy));
  return out_max[0] > out_min[0] && out_max[1] > out_min[1];
}
} // namespace

Filter2Context::Filter2Context(Image& object_image, Image* framebuffer) : object_image_(object_image), framebuffer_image_(framebuffer) {
  object_gpu_ = std::make_unique<GpuImage>(std::max(1u, object_image.width), std::max(1u, object_image.height));
  if(object_gpu_->valid()) object_gpu_->upload(object_image);
  if(framebuffer_image_) {
    framebuffer_gpu_ = std::make_unique<GpuImage>(std::max(1u, framebuffer_image_->width), std::max(1u, framebuffer_image_->height));
    if(framebuffer_gpu_->valid()) framebuffer_gpu_->upload(*framebuffer_image_);
  }
}

Filter2Context::~Filter2Context() {}

void Filter2Context::sync_object_to_image() {
  if(object_gpu_ && object_gpu_->valid()) object_gpu_->readback(object_image_);
}

GpuImage* Filter2Context::resolve(const std::wstring& name_w, bool create_if_missing) {
  const std::string name = wstr_to_utf8(name_w.c_str());
  if(name.empty() || name == "object") return object_gpu_.get();
  if(name == "framebuffer") return framebuffer_gpu_.get();
  if(name == "random") {
    std::lock_guard<std::mutex> lock(g_shared_mtx);
    if(!g_random) g_random = make_random_image(1);
    return g_random.get();
  }
  if(name.rfind("image:", 0) == 0) {
    const std::string path = name.substr(6);
    std::lock_guard<std::mutex> lock(g_shared_mtx);
    auto it = g_image_resources.find(path);
    if(it != g_image_resources.end()) return it->second.get();
    if(!create_if_missing) return nullptr;
    Image img;
    if(!img.load_file(path.c_str())) {
      LOG_F(ERROR, "filter2_compat: 画像を読み込めません: %s", path.c_str());
      return nullptr;
    }
    auto gpu = std::make_unique<GpuImage>(img.width, img.height);
    if(!gpu->valid() || !gpu->upload(img)) return nullptr;
    auto* p                 = gpu.get();
    g_image_resources[path] = std::move(gpu);
    return p;
  }
  const bool is_cache = name.rfind("cache:", 0) == 0;
  auto& table         = is_cache ? g_cache_resources : local_;
  std::unique_lock<std::mutex> cache_lock(g_shared_mtx, std::defer_lock);
  if(is_cache) cache_lock.lock();
  auto it = table.find(name);
  if(it != table.end()) return it->second.get();
  if(!create_if_missing) return nullptr;
  // create_image_resource相当を実装していないため、既定サイズは"object"に合わせる(Phase7の簡略化)
  auto gpu = std::make_unique<GpuImage>(object_gpu_->width(), object_gpu_->height());
  if(!gpu->valid()) return nullptr;
  auto* p     = gpu.get();
  table[name] = std::move(gpu);
  return p;
}

bool Filter2Context::draw_image_to_resource(const wchar_t* dst_resource, const wchar_t* src_resource, float x, float y, float, float, float, float rz, float sx, float sy, float, float alpha) {
  GpuImage* dst = resolve(dst_resource ? dst_resource : L"object", true);
  GpuImage* src = resolve(src_resource ? src_resource : L"object", false);
  if(!dst || !src || !dst->valid() || !src->valid()) return false;
  if(sx == 0 || sy == 0) return true;

  const float rad = rz * (float)M_PI / 180.0f;
  DrawImagePC pc{};
  pc.cx = dst->width() / 2.0f + x, pc.cy = dst->height() / 2.0f + y;
  pc.cos_r = std::cos(rad), pc.sin_r = std::sin(rad);
  pc.sx = sx, pc.sy = sy;
  pc.src_w = (int)src->width(), pc.src_h = (int)src->height();
  pc.am256           = (int)(std::clamp(alpha, 0.0f, 1.0f) * 256.0f + 0.5f);
  pc.blend_mode      = (int)blend_mode_;
  const float half_w = pc.src_w / 2.0f * std::abs(sx), half_h = pc.src_h / 2.0f * std::abs(sy);
  const float diag = std::sqrt(half_w * half_w + half_h * half_h); // 回転を跨いでも収まるようbboxは外接円で確保する(単純化)
  pc.bbox_min[0] = std::max(0, (int)std::floor(pc.cx - diag)), pc.bbox_min[1] = std::max(0, (int)std::floor(pc.cy - diag));
  pc.bbox_max[0] = std::min((int)dst->width(), (int)std::ceil(pc.cx + diag)), pc.bbox_max[1] = std::min((int)dst->height(), (int)std::ceil(pc.cy + diag));
  if(pc.bbox_max[0] <= pc.bbox_min[0] || pc.bbox_max[1] <= pc.bbox_min[1]) return true;

  ComputeParams params;
  params.push       = &pc;
  params.push_size  = sizeof(pc);
  const uint32_t gx = (uint32_t)((pc.bbox_max[0] - pc.bbox_min[0] + 15) / 16), gy = (uint32_t)((pc.bbox_max[1] - pc.bbox_min[1] + 15) / 16);
  std::string err;
  if(!run_compute(draw_image_glsl(), {src}, {dst}, params, gx, gy, 1, &err)) {
    LOG_F(ERROR, "filter2_compat::draw_image: %s", err.c_str());
    return false;
  }
  return true;
}

bool Filter2Context::draw_image(const wchar_t* resource, float x, float y, float z, float rx, float ry, float rz, float sx, float sy, float sz, float alpha) { return draw_image_to_resource(L"object", resource, x, y, z, rx, ry, rz, sx, sy, sz, alpha); }

namespace {
bool poly_num_ok(VERTEX_TYPE t, int n) {
  const bool tri = (int)t <= 4;
  return n > 0 && (n % (tri ? 3 : 4)) == 0;
}
} // namespace

bool Filter2Context::draw_poly_to_resource(const wchar_t* dst_resource, VERTEX_TYPE vertex_type, const void* vertex_list, int vertex_num, const wchar_t* src_resource) {
  if(!vertex_list || !poly_num_ok(vertex_type, vertex_num)) return false;
  GpuImage* dst = resolve(dst_resource ? dst_resource : L"object", true);
  if(!dst || !dst->valid()) return false;
  const int w = (int)dst->width(), h = (int)dst->height();

  const bool textured = vertex_type == VERTEX_TYPE::TRIANGLE_TEXTURE || vertex_type == VERTEX_TYPE::TRIANGLE_TEXTURE_NORM || vertex_type == VERTEX_TYPE::QUAD_TEXTURE || vertex_type == VERTEX_TYPE::QUAD_TEXTURE_NORM;
  const bool is_quad  = (int)vertex_type >= 5;
  const bool has_norm = ((int)vertex_type % 2) == 0;
  GpuImage* src       = textured ? resolve(src_resource ? src_resource : L"object", false) : nullptr;
  if(textured && (!src || !src->valid())) return false;

  // 頂点を(x,y,color-or-uv,a)へ正規化。QUADは(0,1,2)(0,2,3)の2三角形に分割する
  struct V {
    float x, y, a;
    float c0, c1, c2; // color: r,g,b / texture: u,v,(未使用)
  };
  std::vector<V> verts;
  const uint8_t* base = (const uint8_t*)vertex_list;
  size_t stride       = textured ? (has_norm ? sizeof(VERTEX_TEXTURE_NORM) : sizeof(VERTEX_TEXTURE)) : (has_norm ? sizeof(VERTEX_COLOR_NORM) : sizeof(VERTEX_COLOR));
  verts.reserve((size_t)vertex_num);
  for(int i = 0; i < vertex_num; i++) {
    const uint8_t* p = base + stride * (size_t)i;
    V v{};
    if(textured) {
      const auto* t = (const VERTEX_TEXTURE*)p; // NORMも先頭レイアウトは同じ(x,y,z,u,v,a)なので共通で読める
      auto sp       = to_screen(t->x, t->y, w, h);
      v             = {sp.x, sp.y, t->a, t->u, t->v, 0};
    } else {
      const auto* c = (const VERTEX_COLOR*)p;
      auto sp       = to_screen(c->x, c->y, w, h);
      v             = {sp.x, sp.y, c->a, c->r, c->g, c->b};
    }
    verts.push_back(v);
  }

  const int prim_verts = is_quad ? 4 : 3;
  for(int base_i = 0; base_i + prim_verts <= vertex_num; base_i += prim_verts) {
    const int tris                  = is_quad ? 2 : 1;
    static const int quad_idx[2][3] = {{0, 1, 2}, {0, 2, 3}};
    for(int t = 0; t < tris; t++) {
      int i0      = base_i + (is_quad ? quad_idx[t][0] : 0);
      int i1      = base_i + (is_quad ? quad_idx[t][1] : 1);
      int i2      = base_i + (is_quad ? quad_idx[t][2] : 2);
      const V &v0 = verts[i0], &v1 = verts[i1], &v2 = verts[i2];
      const float xy[6] = {v0.x, v0.y, v1.x, v1.y, v2.x, v2.y};
      int32_t bmin[2], bmax[2];
      if(!bbox_from_tri(xy, w, h, bmin, bmax)) continue;
      const uint32_t gx = (uint32_t)((bmax[0] - bmin[0] + 15) / 16), gy = (uint32_t)((bmax[1] - bmin[1] + 15) / 16);
      std::string err;
      bool ok;
      if(textured) {
        PolyTexturePC pc{};
        std::memcpy(pc.xy, xy, sizeof(xy));
        pc.uva[0] = v0.c0, pc.uva[1] = v0.c1, pc.uva[2] = v0.a;
        pc.uva[3] = v1.c0, pc.uva[4] = v1.c1, pc.uva[5] = v1.a;
        pc.uva[6] = v2.c0, pc.uva[7] = v2.c1, pc.uva[8] = v2.a;
        pc.bbox_min[0] = bmin[0], pc.bbox_min[1] = bmin[1], pc.bbox_max[0] = bmax[0], pc.bbox_max[1] = bmax[1];
        pc.src_w = (int)src->width(), pc.src_h = (int)src->height();
        pc.blend_mode = (int)blend_mode_;
        ComputeParams params;
        params.push = &pc, params.push_size = sizeof(pc);
        ok = run_compute(poly_texture_glsl(), {src}, {dst}, params, gx, gy, 1, &err);
      } else {
        PolyColorPC pc{};
        std::memcpy(pc.xy, xy, sizeof(xy));
        pc.rgba[0] = v0.c0, pc.rgba[1] = v0.c1, pc.rgba[2] = v0.c2, pc.rgba[3] = v0.a;
        pc.rgba[4] = v1.c0, pc.rgba[5] = v1.c1, pc.rgba[6] = v1.c2, pc.rgba[7] = v1.a;
        pc.rgba[8] = v2.c0, pc.rgba[9] = v2.c1, pc.rgba[10] = v2.c2, pc.rgba[11] = v2.a;
        pc.bbox_min[0] = bmin[0], pc.bbox_min[1] = bmin[1], pc.bbox_max[0] = bmax[0], pc.bbox_max[1] = bmax[1];
        pc.blend_mode = (int)blend_mode_;
        ComputeParams params;
        params.push = &pc, params.push_size = sizeof(pc);
        ok = run_compute(poly_color_glsl(), {}, {dst}, params, gx, gy, 1, &err);
      }
      if(!ok) {
        LOG_F(ERROR, "filter2_compat::draw_poly: %s", err.c_str());
        return false;
      }
    }
  }
  return true;
}

bool Filter2Context::draw_poly(VERTEX_TYPE vertex_type, const void* vertex_list, int vertex_num, const wchar_t* resource) { return draw_poly_to_resource(L"object", vertex_type, vertex_list, vertex_num, resource); }

namespace {
bool exec_shader_common(const std::string& glsl, const std::vector<GpuImage*>& targets, const std::vector<GpuImage*>& resources, const void* constant, int constant_size, uint32_t gx, uint32_t gy, uint32_t gz) {
  if(targets.empty()) return false;
  for(auto* t : targets)
    if(!t || !t->valid()) return false;
  for(auto* r : resources)
    if(!r || !r->valid()) return false;
  std::vector<GpuImage*> inputs = resources;
  ComputeParams params;
  if(constant && constant_size > 0) {
    if(constant_size <= 128)
      params.push = constant, params.push_size = (uint32_t)constant_size;
    else
      params.ubo = constant, params.ubo_size = (uint32_t)constant_size;
  }
  std::string err;
  if(!run_compute(glsl, inputs, targets, params, gx, gy, gz, &err)) {
    LOG_F(ERROR, "filter2_compat::exec_shader: %s", err.c_str());
    return false;
  }
  return true;
}
} // namespace

bool Filter2Context::exec_pixelshader_data(const char* glsl_src, int src_size, const wchar_t* target, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size) {
  if(!glsl_src || src_size <= 0) return false;
  GpuImage* tgt = resolve(target ? target : L"object", true);
  if(!tgt) return false;
  std::vector<GpuImage*> res;
  for(int i = 0; i < resource_num; i++) {
    GpuImage* r = resolve(resource_list[i], false);
    if(!r) return false;
    res.push_back(r);
  }
  std::string glsl(glsl_src, (size_t)src_size);
  const uint32_t gx = (tgt->width() + 15) / 16, gy = (tgt->height() + 15) / 16;
  return exec_shader_common(glsl, {tgt}, res, constant, constant_size, gx, gy, 1);
}

bool Filter2Context::exec_pixelshader_file(const wchar_t* shader_file, const wchar_t* target, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size) {
  std::string glsl;
  if(!shader_file || !load_glsl_from_file(shader_file, glsl)) return false;
  return exec_pixelshader_data(glsl.data(), (int)glsl.size(), target, resource_list, resource_num, constant, constant_size);
}

bool Filter2Context::exec_computeshader_data(const char* glsl_src, int src_size, const wchar_t** target_list, int target_num, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size, int count_x, int count_y, int count_z) {
  if(!glsl_src || src_size <= 0 || target_num <= 0) return false;
  std::vector<GpuImage*> targets;
  for(int i = 0; i < target_num; i++) {
    GpuImage* t = resolve(target_list[i], true);
    if(!t) return false;
    targets.push_back(t);
  }
  std::vector<GpuImage*> res;
  for(int i = 0; i < resource_num; i++) {
    GpuImage* r = resolve(resource_list[i], false);
    if(!r) return false;
    res.push_back(r);
  }
  std::string glsl(glsl_src, (size_t)src_size);
  return exec_shader_common(glsl, targets, res, constant, constant_size, (uint32_t)std::max(1, count_x), (uint32_t)std::max(1, count_y), (uint32_t)std::max(1, count_z));
}

bool Filter2Context::exec_computeshader_file(const wchar_t* shader_file, const wchar_t** target_list, int target_num, const wchar_t** resource_list, int resource_num, const void* constant, int constant_size, int count_x, int count_y, int count_z) {
  std::string glsl;
  if(!shader_file || !load_glsl_from_file(shader_file, glsl)) return false;
  return exec_computeshader_data(glsl.data(), (int)glsl.size(), target_list, target_num, resource_list, resource_num, constant, constant_size, count_x, count_y, count_z);
}

bool Filter2Context::get_image_data(Vec4b* buffer) {
  if(!buffer || !object_gpu_ || !object_gpu_->valid()) return false;
  Image tmp;
  if(!object_gpu_->readback(tmp)) return false;
  std::memcpy(buffer, tmp.data(), tmp.size_in_bytes());
  return true;
}

bool Filter2Context::set_image_data(const Vec4b* buffer, int width, int height) {
  if(width <= 0 || height <= 0) return false;
  Image tmp(width, height);
  if(buffer)
    std::memcpy(tmp.data(), buffer, tmp.size_in_bytes());
  else
    tmp.fill_rgba(Vec4b(0, 0, 0, 0));
  object_gpu_ = std::make_unique<GpuImage>((uint32_t)width, (uint32_t)height);
  if(!object_gpu_->valid()) return false;
  return object_gpu_->upload(tmp);
}

} // namespace mu::detail
