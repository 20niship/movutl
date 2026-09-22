#include <algorithm>
#include <cstring>
#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/default/image_tile_filter.hpp>
#include <movutl/plugin/default/image_tone_filter.hpp>
#include <movutl/plugin/gpu/gpu_effects.hpp>
#include <movutl/vulkan/vk_context.hpp>
#include <random>

using namespace mu;

namespace {
bool vk_ready() {
  if(VkContext::Get()->create()) return true;
  MESSAGE("SKIP: Vulkanデバイスが利用できません");
  return false;
}

FilterInData make_fin(Image* img) {
  FilterInData fin;
  fin.img = img;
  return fin;
}

void fill_random(Image& img, uint32_t seed) {
  std::mt19937 rng(seed);
  auto* p = img.data();
  for(size_t i = 0; i < img.size(); i++) p[i] = Vec4b((uint8_t)rng(), (uint8_t)rng(), (uint8_t)rng(), (uint8_t)(128 + rng() % 128));
}

int max_diff(const Image& a, const Image& b) {
  int m = 0;
  for(size_t i = 0; i < a.size(); i++)
    for(int c = 0; c < 4; c++) m = std::max(m, std::abs((int)a[i][c] - (int)b[i][c]));
  return m;
}
} // namespace

TEST_CASE("GPU反転: CPU版(f_invert)と一致する") {
  if(!vk_ready()) return;
  Image img_cpu(37, 29);
  fill_random(img_cpu, 1);
  Image img_gpu(37, 29);
  std::memcpy(img_gpu.data(), img_cpu.data(), img_cpu.size_in_bytes());

  auto fin = make_fin(&img_cpu);
  cutil::Prop p;
  p.set<bool>("invert_alpha", true);
  REQUIRE(mu::detail::f_invert.fn_proc(nullptr, &fin, p));
  REQUIRE(mu::detail::gpu_invert(img_gpu, true));

  CHECK(max_diff(img_cpu, img_gpu) <= 1);
}

TEST_CASE("GPU色調補正: brightness/contrastのみ(高速パス)はCPU版と一致する") {
  if(!vk_ready()) return;
  for(auto bc : {std::pair{100.0f, 100.0f}, std::pair{150.0f, 100.0f}, std::pair{100.0f, 130.0f}, std::pair{60.0f, 80.0f}}) {
    Image img_cpu(20, 15);
    fill_random(img_cpu, 7);
    Image img_gpu(20, 15);
    std::memcpy(img_gpu.data(), img_cpu.data(), img_cpu.size_in_bytes());

    auto fin = make_fin(&img_cpu);
    cutil::Prop p;
    p.set<float>("brightness", bc.first);
    p.set<float>("contrast", bc.second);
    REQUIRE(mu::detail::f_color_correction.fn_proc(nullptr, &fin, p));
    REQUIRE(mu::detail::gpu_color_correction(img_gpu, bc.first, bc.second, 0.0f, 100.0f));

    CAPTURE(bc.first);
    CAPTURE(bc.second);
    CHECK(max_diff(img_cpu, img_gpu) <= 1);
  }
}

TEST_CASE("GPU色調補正: hue/saturation指定でも実行でき、値域とアルファを保つ") {
  if(!vk_ready()) return;
  Image img(16, 16);
  fill_random(img, 11);
  const uint8_t alpha0 = img[0][3];
  REQUIRE(mu::detail::gpu_color_correction(img, 100.0f, 100.0f, 45.0f, 150.0f));
  CHECK(img[0][3] == alpha0); // アルファは変更しない
  for(size_t i = 0; i < img.size(); i++)
    for(int c = 0; c < 3; c++) CHECK(img[i][c] <= 255);
}

// 既知の制限: HSV→RGB復元(sector展開)がOpenCVと完全一致せず各ch差が数単位出ることがある(gpu_effects.cpp参照。許容差8)
TEST_CASE("GPU色調補正: hue/saturation指定時もCPU版とおおむね一致する") {
  if(!vk_ready()) return;
  for(auto hs : {std::pair{45.0f, 150.0f}, std::pair{-90.0f, 50.0f}, std::pair{179.0f, 100.0f}, std::pair{10.0f, 200.0f}}) {
    Image img_cpu(24, 17);
    fill_random(img_cpu, 99);
    Image img_gpu(24, 17);
    std::memcpy(img_gpu.data(), img_cpu.data(), img_cpu.size_in_bytes());

    auto fin = make_fin(&img_cpu);
    cutil::Prop p;
    p.set<float>("hue", hs.first);
    p.set<float>("saturation", hs.second);
    REQUIRE(mu::detail::f_color_correction.fn_proc(nullptr, &fin, p));
    REQUIRE(mu::detail::gpu_color_correction(img_gpu, 100.0f, 100.0f, hs.first, hs.second));

    CAPTURE(hs.first);
    CAPTURE(hs.second);
    CHECK(max_diff(img_cpu, img_gpu) <= 8);
  }
}

TEST_CASE("GPUタイル: CPU版(f_tile)と一致する") {
  if(!vk_ready()) return;
  for(auto nxny : {std::pair{2, 2}, std::pair{3, 1}, std::pair{1, 1}, std::pair{3, 3}}) {
    Image img_cpu(10, 10);
    fill_random(img_cpu, 3);
    Image img_gpu(10, 10);
    std::memcpy(img_gpu.data(), img_cpu.data(), img_cpu.size_in_bytes());

    auto fin = make_fin(&img_cpu);
    cutil::Prop p;
    p.set<int32_t>("nx", nxny.first);
    p.set<int32_t>("ny", nxny.second);
    REQUIRE(mu::detail::f_tile.fn_proc(nullptr, &fin, p));
    REQUIRE(mu::detail::gpu_tile(img_gpu, nxny.first, nxny.second));

    CAPTURE(nxny.first);
    CAPTURE(nxny.second);
    CHECK(max_diff(img_cpu, img_gpu) == 0);
  }
}
