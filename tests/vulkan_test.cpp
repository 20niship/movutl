#include <cstdlib>
#include <cstring>
#include <doctest/doctest.h>
#include <movutl/vulkan/vk_context.hpp>
#include <movutl/vulkan/vk_image.hpp>
#include <random>

using namespace mu;

namespace {
// Vulkanデバイスが無い環境(CI等)ではテストをSKIPする
bool vk_ready() {
  if(VkContext::Get()->create()) return true;
  MESSAGE("SKIP: Vulkanデバイスが利用できません");
  return false;
}

void fill_random(Image& img, uint32_t seed) {
  std::mt19937 rng(seed);
  auto* p = reinterpret_cast<uint8_t*>(img.data());
  for(size_t i = 0; i < img.size_in_bytes(); i++) p[i] = (uint8_t)rng();
}

bool roundtrip(int w, int h) {
  Image src(w, h);
  fill_random(src, w * 31 + h);
  GpuImage gpu(w, h);
  if(!gpu.upload(src)) return false;
  Image dst;
  if(!gpu.readback(dst)) return false;
  return dst.width == (unsigned)w && dst.height == (unsigned)h && std::memcmp(src.data(), dst.data(), src.size_in_bytes()) == 0;
}
} // namespace

TEST_CASE("VkContext: ヘッドレスで生成できる") {
  if(!vk_ready()) return;
  auto* c = VkContext::Get();
  CHECK(c->valid());
  CHECK(c->device() != VK_NULL_HANDLE);
  CHECK(c->create()); // 2回目は生成済みでtrue
}

TEST_CASE("GpuImage: upload→readbackで完全一致") {
  if(!vk_ready()) return;
  CHECK(roundtrip(256, 128));
  CHECK(roundtrip(1, 1));
  CHECK(roundtrip(37, 53)); // 非2冪
  CHECK(roundtrip(4096, 2160));
}

TEST_CASE("GpuImage: サイズ不一致のuploadは失敗する") {
  if(!vk_ready()) return;
  Image src(8, 8);
  GpuImage gpu(16, 16);
  CHECK_FALSE(gpu.upload(src));
}
