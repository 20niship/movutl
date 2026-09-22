#include <doctest/doctest.h>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
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

// dst全画素・RGBAの各チャンネル差がtol以下ならtrue
bool images_close(const Image& a, const Image& b, int tol) {
  if(a.width != b.width || a.height != b.height) return false;
  for(size_t i = 0; i < a.size(); i++) {
    for(int c = 0; c < 4; c++)
      if(std::abs((int)a[i][c] - (int)b[i][c]) > tol) return false;
  }
  return true;
}

// CPU/Vulkan両方でレンダリングし、差がtol以下であることを確認する
void check_cpu_vulkan_match(Composition* comp, int frame, int tol = 2) {
  CPURenderer cpu;
  Ref<Image> cpu_out;
  REQUIRE(cpu.render_frame(comp, frame, cpu_out));

  VulkanRenderer gpu;
  Ref<Image> gpu_out;
  REQUIRE(gpu.render_frame(comp, frame, gpu_out));

  CHECK(images_close(*cpu_out, *gpu_out, tol));
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
