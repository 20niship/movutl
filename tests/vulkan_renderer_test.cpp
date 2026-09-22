#include <doctest/doctest.h>
#include <movutl/asset/camera.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/group.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/scene_change.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/vulkan/vk_context.hpp>
#include <movutl/vulkan/vulkan_renderer.hpp>
#include <thread>
#include <vector>

using namespace mu;

namespace {
bool vk_ready() {
  if(VkContext::Get()->create()) return true;
  MESSAGE("SKIP: Vulkanデバイスが利用できません");
  return false;
}

Ref<Composition> make_test_comp(int w, int h) { return cutil::make_ref<Composition>("test", w, h, 30); }

// dst全画素・RGBAの各チャンネル差がtol以下ならtrue。max_bad_pixelsは、回転四角形の逆行列をfloatへ落とすことによる辺上の数px食い違いの許容数
bool images_close(const Image& a, const Image& b, int tol, int max_bad_pixels = 0) {
  if(a.width != b.width || a.height != b.height) return false;
  int bad = 0;
  for(size_t i = 0; i < a.size(); i++) {
    for(int c = 0; c < 4; c++)
      if(std::abs((int)a[i][c] - (int)b[i][c]) > tol) {
        if(++bad > max_bad_pixels) return false;
        break;
      }
  }
  return true;
}

// CPU/Vulkan両方でレンダリングし、差がtol以下であることを確認する
void check_cpu_vulkan_match(Composition* comp, int frame, int tol = 2, int max_bad_pixels = 0) {
  CPURenderer cpu;
  Ref<Image> cpu_out;
  REQUIRE(cpu.render_frame(comp, frame, cpu_out));

  VulkanRenderer gpu;
  Ref<Image> gpu_out;
  REQUIRE(gpu.render_frame(comp, frame, gpu_out));

  CHECK(images_close(*cpu_out, *gpu_out, tol, max_bad_pixels));
}
} // namespace

TEST_CASE("VulkanRenderer: Shape単体はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp     = make_test_comp(64, 64);
  auto rect     = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_   = Vec2(30, 20);
  rect->color_  = Vec4b(10, 200, 30, 255);
  rect->fstart_ = 0;
  rect->fend_   = 5;
  comp->insert_entity(rect, 0);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: Image単体はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp = make_test_comp(40, 40);
  auto img  = Image::Create("i", 20, 16);
  img->fill_rgba(Vec4b(80, 120, 200, 255));
  img->fstart_ = 0;
  img->fend_   = 5;
  comp->insert_entity(img, 0);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: Text単体はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp    = make_test_comp(120, 40);
  auto txt     = TextEntt::Create("Hi");
  txt->fstart_ = 0;
  txt->fend_   = 5;
  comp->insert_entity(txt, 0);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: 複数レイヤーの重なりはCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp = make_test_comp(50, 50);
  auto a    = ShapeEntt::Create("a", ShapeType_Rect);
  a->size_ = Vec2(50, 50), a->color_ = Vec4b(255, 0, 0, 255), a->fstart_ = 0, a->fend_ = 5;
  auto b   = ShapeEntt::Create("b", ShapeType_Circle);
  b->size_ = Vec2(30, 30), b->color_ = Vec4b(0, 255, 0, 180), b->fstart_ = 0, b->fend_ = 5;
  comp->insert_entity(a, 0);
  comp->insert_entity(b, 1);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: alpha 0/0.5/1はCPURendererと一致する") {
  if(!vk_ready()) return;
  for(float alpha : {0.0f, 0.5f, 1.0f}) {
    auto comp      = make_test_comp(40, 40);
    comp->bg_color = 0;
    auto rect      = ShapeEntt::Create("r", ShapeType_Rect);
    rect->size_    = Vec2(30, 30);
    rect->color_   = Vec4b(255, 128, 0, 255);
    rect->alpha_   = alpha;
    rect->fstart_  = 0;
    rect->fend_    = 5;
    comp->insert_entity(rect, 0);
    check_cpu_vulkan_match(comp.get(), 0);
  }
}

TEST_CASE("VulkanRenderer: blend全10種はCPURendererと一致する") {
  if(!vk_ready()) return;
  for(int b = 0; b <= 9; b++) {
    auto comp = make_test_comp(30, 30);
    auto bg   = ShapeEntt::Create("bg", ShapeType_Rect);
    bg->size_ = Vec2(30, 30), bg->color_ = Vec4b(120, 60, 200, 255), bg->fstart_ = 0, bg->fend_ = 5;
    auto fg   = ShapeEntt::Create("fg", ShapeType_Rect);
    fg->size_ = Vec2(30, 30), fg->color_ = Vec4b(40, 220, 90, 200), fg->fstart_ = 0, fg->fend_ = 5;
    fg->blend_ = (BlendType)b;
    comp->insert_entity(bg, 0);
    comp->insert_entity(fg, 1);
    CAPTURE(b);
    check_cpu_vulkan_match(comp.get(), 0);
  }
}

TEST_CASE("VulkanRenderer: 移動/回転/拡大/アンカーはCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp       = make_test_comp(80, 80);
  auto rect       = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_     = Vec2(20, 10);
  rect->color_    = Vec4b(200, 50, 50, 255);
  rect->pos_      = Vec3(10, -5, 0);
  rect->rotation_ = 35.0f;
  rect->scale_    = 150.0f;
  rect->anchor_   = Vec3(5, 0, 0);
  rect->fstart_   = 0;
  rect->fend_     = 5;
  comp->insert_entity(rect, 0);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: 背景色/transparent_bgはCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp      = make_test_comp(20, 20);
  comp->bg_color = 0xFF3366CCu;
  auto rect      = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_    = Vec2(10, 10);
  rect->color_   = Vec4b(0, 255, 255, 255);
  rect->fstart_  = 0;
  rect->fend_    = 5;
  comp->insert_entity(rect, 0);

  CPURenderer cpu;
  Ref<Image> cpu_out, cpu_out_t;
  REQUIRE(cpu.render_frame(comp.get(), 0, cpu_out, false));
  REQUIRE(cpu.render_frame(comp.get(), 0, cpu_out_t, true));

  VulkanRenderer gpu;
  Ref<Image> gpu_out, gpu_out_t;
  REQUIRE(gpu.render_frame(comp.get(), 0, gpu_out, false));
  REQUIRE(gpu.render_frame(comp.get(), 0, gpu_out_t, true));

  CHECK(images_close(*cpu_out, *gpu_out, 2));
  CHECK(images_close(*cpu_out_t, *gpu_out_t, 2));
}

TEST_CASE("VulkanRenderer: 空compositionと1x1でも動作する") {
  if(!vk_ready()) return;
  auto empty_comp = make_test_comp(30, 30);
  check_cpu_vulkan_match(empty_comp.get(), 0);

  auto tiny   = make_test_comp(1, 1);
  auto rect   = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_ = Vec2(1, 1), rect->color_ = Vec4b(1, 2, 3, 255), rect->fstart_ = 0, rect->fend_ = 5;
  tiny->insert_entity(rect, 0);
  check_cpu_vulkan_match(tiny.get(), 0);
}

TEST_CASE("VulkanRenderer: 複数インスタンスから同時にrender_frameを呼んでも結果が同一") {
  if(!vk_ready()) return;
  auto comp   = make_test_comp(48, 48);
  auto rect   = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_ = Vec2(30, 20), rect->color_ = Vec4b(90, 150, 210, 255), rect->fstart_ = 0, rect->fend_ = 5;
  comp->insert_entity(rect, 0);

  std::vector<Ref<Image>> results(4);
  std::vector<std::thread> threads;
  for(int i = 0; i < 4; i++) {
    threads.emplace_back([&, i] {
      VulkanRenderer r;
      r.render_frame(comp.get(), 0, results[i]);
    });
  }
  for(auto& t : threads) t.join();
  for(int i = 1; i < 4; i++) CHECK(images_close(*results[0], *results[i], 0));
}

TEST_CASE("VulkanRenderer: X軸/Y軸回転(透視)はCPURendererと一致する") {
  if(!vk_ready()) return;
  for(auto rxy : {std::pair{30.0f, 0.0f}, std::pair{0.0f, -45.0f}, std::pair{20.0f, 60.0f}}) {
    auto comp     = make_test_comp(80, 80);
    auto rect     = ShapeEntt::Create("r", ShapeType_Rect);
    rect->size_   = Vec2(40, 30);
    rect->color_  = Vec4b(220, 90, 40, 255);
    rect->rot_x_  = rxy.first;
    rect->rot_y_  = rxy.second;
    rect->fstart_ = 0;
    rect->fend_   = 5;
    comp->insert_entity(rect, 0);
    CAPTURE(rxy.first);
    CAPTURE(rxy.second);
    check_cpu_vulkan_match(comp.get(), 0, 2, 2);
  }
}

TEST_CASE("VulkanRenderer: グループの親変換はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp      = make_test_comp(100, 100);
  comp->bg_color = 0;
  auto g         = GroupEntt::Create("g");
  g->pos_        = Vec3(15, -10, 0);
  g->rotation_   = 25.0f;
  g->scale_      = 130.0f;
  g->fstart_     = 0;
  g->fend_       = 10;
  auto rect      = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_    = Vec2(20, 20);
  rect->color_   = Vec4b(50, 180, 220, 255);
  rect->pos_     = Vec3(10, 5, 0);
  rect->fstart_  = 0;
  rect->fend_    = 10;
  comp->insert_entity(g, 0);
  comp->insert_entity(rect, 1);
  check_cpu_vulkan_match(comp.get(), 0, 2, 2);
}

TEST_CASE("VulkanRenderer: カメラの視点変換はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp          = make_test_comp(100, 100);
  comp->bg_color     = 0;
  auto cam           = Camera3D::Create("cam");
  cam->pos_          = Vec3(30, 0, -400);
  cam->fstart_       = 0;
  cam->fend_         = 10;
  auto rect          = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_        = Vec2(20, 20);
  rect->color_       = Vec4b(240, 200, 30, 255);
  rect->camera_ctrl_ = true;
  rect->fstart_      = 0;
  rect->fend_        = 10;
  comp->insert_entity(cam, 0);
  comp->insert_entity(rect, 1);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: シーンチェンジ(フェード)はCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp   = make_test_comp(60, 60);
  auto sc     = SceneChangeEntt::Create("sc");
  sc->type_   = SceneChangeType_Fade;
  sc->fstart_ = 5;
  sc->fend_   = 15;
  auto a      = ShapeEntt::Create("a", ShapeType_Rect);
  a->size_ = Vec2(60, 60), a->color_ = Vec4b(255, 0, 0, 255), a->fstart_ = 0, a->fend_ = 5;
  auto b   = ShapeEntt::Create("b", ShapeType_Rect);
  b->size_ = Vec2(60, 60), b->color_ = Vec4b(0, 0, 255, 255), b->fstart_ = 5, b->fend_ = 20;
  comp->insert_entity(sc, 0);
  comp->insert_entity(a, 1);
  comp->insert_entity(b, 1);
  for(int f : {5, 10, 15}) {
    CAPTURE(f);
    check_cpu_vulkan_match(comp.get(), f);
  }
}

TEST_CASE("VulkanRenderer: 上のオブジェクトでクリッピングはCPURendererと一致する") {
  if(!vk_ready()) return;
  auto comp         = make_test_comp(60, 60);
  comp->bg_color    = 0;
  auto bottom       = ShapeEntt::Create("bottom", ShapeType_Rect);
  bottom->pos_      = Vec3(-15, 0, 0);
  bottom->size_     = Vec2(30, 60);
  bottom->color_    = Vec4b(255, 0, 0, 255);
  bottom->fstart_   = 0;
  bottom->fend_     = 10;
  auto top          = ShapeEntt::Create("top", ShapeType_Rect);
  top->pos_         = Vec3(0, 0, 0);
  top->size_        = Vec2(60, 60);
  top->color_       = Vec4b(0, 0, 255, 255);
  top->fstart_      = 0;
  top->fend_        = 10;
  top->clipping_up_ = true;
  comp->insert_entity(bottom, 0);
  comp->insert_entity(top, 1);
  check_cpu_vulkan_match(comp.get(), 0);
}

TEST_CASE("VulkanRenderer: readbackはrender_frame1回につき1回だけ発生する(ゼロコピー表示は未実装。計測用)") {
  if(!vk_ready()) return;
  auto comp   = make_test_comp(32, 32);
  auto rect   = ShapeEntt::Create("r", ShapeType_Rect);
  rect->size_ = Vec2(10, 10), rect->color_ = Vec4b(1, 2, 3, 255), rect->fstart_ = 0, rect->fend_ = 5;
  comp->insert_entity(rect, 0);

  VulkanRenderer gpu;
  Ref<Image> out;
  const uint64_t before = gpu_readback_count().load();
  REQUIRE(gpu.render_frame(comp.get(), 0, out));
  const uint64_t after = gpu_readback_count().load();
  // render_current_frame_main_threadがRef<Image>を返す設計のため、ゼロコピー化には戻り値の型変更が要る(次フェーズ)
  CHECK(after - before == 1);
}
