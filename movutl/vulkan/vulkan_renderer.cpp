#include <cmath>
#include <movutl/asset/camera.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/scene_change.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/render2d/composite_ops.hpp>
#include <movutl/render2d/renderer_registry.hpp>
#include <movutl/vulkan/gpu_compute.hpp>
#include <movutl/vulkan/vulkan_renderer.hpp>
#include <opencv2/opencv.hpp>
#include <set>

namespace mu {

namespace {
// dst(x,y)->src(u,v)の射影行列[m0 m1 m2; m3 m4 m5; m6 m7 m8]。u=(m0 x+m1 y+m2)/w, v=(m3 x+m4 y+m5)/w, w=m6 x+m7 y+m8。CPU版と同じ数式(image.cpp)に合わせる
struct PlaceXform {
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0; // dst bbox(半開区間)。空ならx1<=x0
};

// 分岐と数式はmovutl/asset/image.cppのImage::place/transform_to/drawquadに合わせてある
PlaceXform build_xform(const Image& src, int dst_w, int dst_h, const Placement& pl) {
  PlaceXform r;
  double sx = pl.scale_x, sy = pl.scale_y;
  if(pl.aspect > 0)
    sx *= 1.0 - pl.aspect;
  else
    sy *= 1.0 + pl.aspect;
  if(sx == 0 || sy == 0) return r; // r.x1<=r.x0のまま(何も描かない)
  const double px = pl.anchor_x * sx, py = pl.anchor_y * sy;
  const double ox = dst_w / 2.0 + pl.x, oy = dst_h / 2.0 + pl.y;
  const double rad = pl.rot_z * M_PI / 180.0;
  const double c = std::cos(rad), sn = std::sin(rad);
  const double src_cx = src.width / 2.0, src_cy = src.height / 2.0;

  if(pl.rot_x == 0.0 && pl.rot_y == 0.0) {
    const double cx = ox - (px * c - py * sn);
    const double cy = oy - (px * sn + py * c);
    if(sx == 1.0 && sy == 1.0 && pl.rot_z == 0.0) {
      // Image::place()のcopyto(pmin)経路と同じ整数オフセットで合わせる(floor(cx-src_cx)は一般式のfloor(u)と偶数/奇数サイズで1px食い違いうる)
      const int px0 = (int)std::floor(cx - src_cx), py0 = (int)std::floor(cy - src_cy);
      r.m[0] = 1, r.m[1] = 0, r.m[2] = -px0;
      r.m[3] = 0, r.m[4] = 1, r.m[5] = -py0;
      r.m[6] = 0, r.m[7] = 0, r.m[8] = 1;
      r.x0 = std::max(0, px0), r.x1 = std::min(dst_w, px0 + (int)src.width);
      r.y0 = std::max(0, py0), r.y1 = std::min(dst_h, py0 + (int)src.height);
      return r;
    }
    r.m[0] = c / sx, r.m[1] = sn / sx, r.m[2] = src_cx - cx * c / sx - cy * sn / sx;
    r.m[3] = -sn / sy, r.m[4] = c / sy, r.m[5] = src_cy + cx * sn / sy - cy * c / sy;
    r.m[6] = 0, r.m[7] = 0, r.m[8] = 1;

    const double half_w = src_cx * std::abs(sx), half_h = src_cy * std::abs(sy);
    const double corner_x[4] = {-half_w, half_w, -half_w, half_w}, corner_y[4] = {-half_h, -half_h, half_h, half_h};
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for(int i = 0; i < 4; i++) {
      const double bx = cx + corner_x[i] * c - corner_y[i] * sn, by = cy + corner_x[i] * sn + corner_y[i] * c;
      min_x = std::min(min_x, bx), max_x = std::max(max_x, bx);
      min_y = std::min(min_y, by), max_y = std::max(max_y, by);
    }
    r.x0 = std::max(0, (int)std::floor(min_x)), r.x1 = std::min(dst_w, (int)std::ceil(max_x));
    r.y0 = std::max(0, (int)std::floor(min_y)), r.y1 = std::min(dst_h, (int)std::ceil(max_y));
    return r;
  }

  // X/Y軸回転(透視): Image::place()と同じ四隅計算からcv::getPerspectiveTransformで順変換行列を作り、逆行列をdst->srcマッピングに使う
  constexpr double kCameraDistance = 1024.0;
  const double ax = pl.rot_x * M_PI / 180.0, ay = pl.rot_y * M_PI / 180.0;
  const double cax = std::cos(ax), sax = std::sin(ax), cay = std::cos(ay), say = std::sin(ay);
  const double hw = src.width / 2.0 * sx, hh = src.height / 2.0 * sy;
  const double cxs[4] = {-hw, hw, -hw, hw}, cys[4] = {-hh, -hh, hh, hh};
  cv::Point2f dst_pts[4];
  for(int i = 0; i < 4; i++) {
    const double xx = cxs[i] - px, yy = cys[i] - py;
    const double y1 = yy * cax, z1 = yy * sax;
    const double x2 = xx * cay + z1 * say, z2 = -xx * say + z1 * cay;
    const double x3 = x2 * c - y1 * sn, y3 = x2 * sn + y1 * c;
    const double persp = kCameraDistance / std::max(kCameraDistance + z2, 1.0);
    dst_pts[i]         = cv::Point2f((float)(ox + x3 * persp), (float)(oy + y3 * persp));
  }
  const float sw = std::max(1.0f, (float)src.width - 1), sh = std::max(1.0f, (float)src.height - 1);
  cv::Point2f src_pts[4] = {{0, 0}, {sw, 0}, {0, sh}, {sw, sh}};
  cv::Mat fwd            = cv::getPerspectiveTransform(src_pts, dst_pts);
  cv::Mat inv;
  if(!cv::invert(fwd, inv)) return r; // 退化した四角形は何も描かない(CPU版drawquadと同じ扱い)
  const double* im = inv.ptr<double>();
  for(int i = 0; i < 9; i++) r.m[i] = im[i];

  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  for(auto& p : dst_pts) min_x = std::min(min_x, p.x), max_x = std::max(max_x, p.x), min_y = std::min(min_y, p.y), max_y = std::max(max_y, p.y);
  r.x0 = std::max(0, (int)std::floor(min_x)), r.x1 = std::min(dst_w, (int)std::ceil(max_x));
  r.y0 = std::max(0, (int)std::floor(min_y)), r.y1 = std::min(dst_h, (int)std::ceil(max_y));
  return r;
}

// movutl/asset/image.cppのblend_channel/blend_pixel_tと同じ整数式(0=Alpha,1=Add,...9=HardLight)をcomputeで再現する
constexpr const char* kCompositeGlsl = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform readonly image2D srcImg;
layout(binding=1, rgba8) uniform image2D dstImg;
// vec/ivec型はstd430で8バイト境界に整列されC++側の構造体とズレるため、push constantはスカラーのみで組む
layout(push_constant) uniform PC {
  float m0,m1,m2,m3,m4,m5,m6,m7,m8;
  int bbox_min_x, bbox_min_y;
  int bbox_max_x, bbox_max_y;
  int src_w, src_h;
  int am256;
  int blend_mode;
} pc;

int div255(int x) { return (x + 128 + ((x + 128) >> 8)) >> 8; }

int blend_channel(int mode, int d, int s) {
  if(mode == 1) return min(255, d + s);
  if(mode == 2) return max(0, d - s);
  if(mode == 3) return div255(d * s);
  if(mode == 4) return s > 0 ? min(255, d * 255 / s) : 255;
  if(mode == 5) return d + s - d * s / 255;
  if(mode == 6) return d < 128 ? 2 * d * s / 255 : 255 - 2 * (255 - d) * (255 - s) / 255;
  if(mode == 7) return min(d, s);
  if(mode == 8) return max(d, s);
  if(mode == 9) return s < 128 ? 2 * d * s / 255 : 255 - 2 * (255 - d) * (255 - s) / 255;
  return s;
}

void main() {
  ivec2 dp = ivec2(gl_GlobalInvocationID.xy) + ivec2(pc.bbox_min_x, pc.bbox_min_y);
  if(dp.x >= pc.bbox_max_x || dp.y >= pc.bbox_max_y) return;
  float x = float(dp.x), y = float(dp.y);
  float w = pc.m6 * x + pc.m7 * y + pc.m8;
  if(abs(w) < 1e-9) return;
  float u = (pc.m0 * x + pc.m1 * y + pc.m2) / w;
  float v = (pc.m3 * x + pc.m4 * y + pc.m5) / w;
  int sx = int(floor(u)), sy = int(floor(v));
  if(sx < 0 || sx >= pc.src_w || sy < 0 || sy >= pc.src_h) return;

  ivec4 s255 = ivec4(round(imageLoad(srcImg, ivec2(sx, sy)) * 255.0));
  int a = ((s255.a + (s255.a >> 7)) * pc.am256) >> 8;
  if(a <= 0) return;
  if(pc.blend_mode == 0 && a >= 256) {
    imageStore(dstImg, dp, vec4(s255) / 255.0);
    return;
  }
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

struct PushConst {
  float m[9];
  int32_t bbox_min[2];
  int32_t bbox_max[2];
  int32_t src_size[2];
  int32_t am256;
  int32_t blend_mode;
};

inline int alpha_to_256(float a) { return (int)(std::clamp(a, 0.0f, 1.0f) * 256.0f + 0.5f); }
} // namespace

VulkanRenderer::VulkanRenderer() {
  ready_ = VkContext::Get()->create();
  if(!ready_) LOG_F(WARNING, "VulkanRenderer: Vulkanが利用できないためCPURendererへ委譲します");
}

void VulkanRenderer::ensure_target(int w, int h) {
  if(gpu_out_ && (int)gpu_out_->width() == w && (int)gpu_out_->height() == h) return;
  gpu_out_ = std::make_unique<GpuImage>((uint32_t)w, (uint32_t)h);
}

bool VulkanRenderer::place(const Image& src, Image* /*target*/, const Placement& pl) {
  if(src.empty() || !gpu_out_ || !gpu_out_->valid()) return false;
  PlaceXform xf = build_xform(src, (int)gpu_out_->width(), (int)gpu_out_->height(), pl);
  if(xf.x1 <= xf.x0 || xf.y1 <= xf.y0) return true; // 完全に画面外(何も描かない)

  GpuImage src_gpu(src.width, src.height);
  if(!src_gpu.valid() || !src_gpu.upload(const_cast<Image&>(src))) return false;

  PushConst pc{};
  for(int i = 0; i < 9; i++) pc.m[i] = (float)xf.m[i];
  pc.bbox_min[0] = xf.x0, pc.bbox_min[1] = xf.y0;
  pc.bbox_max[0] = xf.x1, pc.bbox_max[1] = xf.y1;
  pc.src_size[0] = (int)src.width, pc.src_size[1] = (int)src.height;
  pc.am256      = alpha_to_256(pl.alpha);
  pc.blend_mode = (int)pl.blend;

  ComputeParams params;
  params.push       = &pc;
  params.push_size  = sizeof(pc);
  const uint32_t gx = (uint32_t)((xf.x1 - xf.x0 + 15) / 16), gy = (uint32_t)((xf.y1 - xf.y0 + 15) / 16);
  std::string err;
  if(!run_compute(kCompositeGlsl, {&src_gpu}, {gpu_out_.get()}, params, gx, gy, 1, &err)) {
    LOG_F(ERROR, "VulkanRenderer::place: %s", err.c_str());
    return false;
  }
  return true;
}

bool VulkanRenderer::render_frame(Composition* comp, int frame, Ref<Image>& out, bool transparent_bg) {
  MOVUTL_ZONE_SCOPED_N("VulkanRenderer::render_frame");
  MU_ASSERT(comp != nullptr);
  if(!ready_) return fallback_.render_frame(comp, frame, out, transparent_bg);

  const int w = (int)comp->size[0], h = (int)comp->size[1];
  ensure_target(w, h);
  if(!gpu_out_->valid()) return fallback_.render_frame(comp, frame, out, transparent_bg);

  {
    Image bg(w, h);
    if(transparent_bg) {
      bg.fill_rgba(Vec4b(0, 0, 0, 0));
    } else {
      uint32_t c = (uint32_t)comp->bg_color;
      bg.fill_rgba(Vec4b{(unsigned char)(c & 0xFF), (unsigned char)((c >> 8) & 0xFF), (unsigned char)((c >> 16) & 0xFF), (unsigned char)((c >> 24) & 0xFF)});
    }
    gpu_out_->upload(bg);
  }

  const auto layered = comp->get_layered_entities();
  std::set<int> mask_needed;
  for(auto& [li, ce] : layered)
    if(ce->clipping_up_ && ce->visible(frame)) mask_needed.insert(li - 1);
  Image mask;
  int mask_layer = -1000;
  Image cpu_bridge(w, h);   // GPU⇄CPU橋渡し用の再利用バッファ(Framebuffer/CustomObject/scene_change/clipping_up)
  Image dummy_target(1, 1); // GPU合成時はcomposite()内で無視される。非nullチェック用のプレースホルダ

  for(auto& [layer_i, e] : layered) {
    std::unique_lock<std::mutex> lock_try(e->mtx, std::defer_lock);
    lock_try.lock();
    if(!e->visible(frame)) continue;
    e->apply_animated_props(frame);

    GroupXform parent;
    bool has_parent = false;
    for(auto& [gl, g] : layered) {
      if(gl >= layer_i) break;
      if(g->getType() != EntityType_Group || !g->visible(frame)) continue;
      auto* ge = static_cast<GroupEntt*>(g.get());
      if(!ge->affects(gl, layer_i)) continue;
      parent     = parent.compose(ge->local_xform());
      has_parent = true;
    }
    if(e->camera_ctrl_) {
      const Camera3D* cam = nullptr;
      for(auto& [cl, c] : layered) {
        if(cl >= layer_i) break;
        if(c->getType() != EntityType_Camera || !c->visible(frame)) continue;
        auto* ce = static_cast<Camera3D*>(c.get());
        if(ce->affects(cl, layer_i)) cam = ce;
      }
      if(cam) {
        parent     = cam->view_xform().compose(parent);
        has_parent = true;
      }
    }
    GroupXformScope parent_scope(has_parent ? &parent : nullptr);

    const SceneChangeEntt* sc = nullptr;
    for(auto& [sl, s] : layered) {
      if(sl >= layer_i) break;
      if(s->getType() != EntityType_SceneChange || !s->visible(frame)) continue;
      auto* se = static_cast<SceneChangeEntt*>(s.get());
      if(se->affects(sl, layer_i)) {
        sc = se;
        break;
      }
    }

    // target実内容(合成済み画面)を読み書きする経路はCPUに橋渡しする(GPU合成の逆行列は「未合成のsrc画像→dst配置」しか扱えない)
    const bool needs_cpu_bridge = sc && e->getType() != EntityType_SceneChange;
    const bool needs_clip_cpu   = !needs_cpu_bridge && e->clipping_up_ && mask_layer == layer_i - 1;
    const bool needs_rw_target  = !needs_cpu_bridge && !needs_clip_cpu && (e->getType() == EntityType_Framebuffer || e->getType() == EntityType_Custom);

    if(needs_cpu_bridge || needs_clip_cpu || needs_rw_target) {
      gpu_out_->readback(cpu_bridge);
      if(needs_cpu_bridge) {
        Entity* a = nullptr;
        for(auto& [al, ae] : layered)
          if(al == layer_i && ae.get() != e.get() && ae->fend_ < e->fstart_ && (!a || ae->fend_ > a->fend_)) a = ae.get();
        composite_scene_change(comp, a, e.get(), sc, &cpu_bridge, frame);
      } else if(needs_clip_cpu) {
        composite_clipping_up(comp, e.get(), mask, &cpu_bridge, frame);
      } else {
        e->render(comp, &cpu_bridge, frame);
      }
      gpu_out_->upload(cpu_bridge);
    } else {
      GpuCompositeSinkScope sink_scope(this);
      e->render(comp, &dummy_target, frame);
    }

    if(mask_needed.count(layer_i)) {
      // 直後のレイヤーのクリッピング用に、このオブジェクト単体のアルファをCPUで保持する(GPU sinkは無効のまま=通常のCPU合成)
      mask.resize(Vec2d(w, h));
      mask.fill_rgba(Vec4b(0, 0, 0, 0));
      e->render(comp, &mask, frame);
      mask_layer = layer_i;
    }
  }

  if(!out) out = cutil::make_ref<Image>(w, h);
  gpu_out_->readback(*out);
  out->dirty();
  return true;
}

namespace {
bool g_vulkan_registered = [] {
  register_renderer("vulkan", [] { return std::make_unique<VulkanRenderer>(); });
  return true;
}();
} // namespace

} // namespace mu
