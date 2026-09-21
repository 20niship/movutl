#include <cmath>
#include <cstring>
#include <doctest/doctest.h>
#include <movutl/vulkan/gpu_compute.hpp>
#include <movutl/vulkan/shader_util.hpp>
#include <random>

using namespace mu;

namespace {
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

const char* kInvert = R"(#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(set=0, binding=0, rgba8) uniform readonly image2D src;
layout(set=0, binding=1, rgba8) uniform writeonly image2D dst;
void main() {
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(any(greaterThanEqual(p, imageSize(dst)))) return;
  vec4 c = imageLoad(src, p);
  imageStore(dst, p, vec4(1.0 - c.rgb, c.a));
}
)";
} // namespace

TEST_CASE("run_compute_image: 反転がCPU期待値と一致する") {
  if(!vk_ready()) return;
  Image src(100, 70); // 16の倍数でないサイズ
  fill_random(src, 1);
  Image dst(100, 70);
  std::string err;
  REQUIRE_MESSAGE(run_compute_image(kInvert, {&src}, dst, {}, &err), err);
  auto* s      = reinterpret_cast<uint8_t*>(src.data());
  auto* d      = reinterpret_cast<uint8_t*>(dst.data());
  int max_diff = 0;
  for(size_t i = 0; i < src.size_in_bytes(); i++) {
    int expect = (i % 4 == 3) ? s[i] : 255 - s[i];
    max_diff   = std::max(max_diff, std::abs(expect - (int)d[i]));
  }
  CHECK(max_diff <= 1);
}

TEST_CASE("compile_glsl: 不正GLSLは行番号付きエラー文字列を返す") {
  auto r = compile_glsl("#version 450\nlayout(local_size_x=1) in;\nvoid main() { undefined_fn(); }\n");
  CHECK_FALSE(r.ok);
  CHECK(r.error.find("ERROR") != std::string::npos);
  CHECK(r.error.find("0:3") != std::string::npos);

  if(!vk_ready()) return;
  Image src(4, 4), dst(4, 4);
  std::string err;
  CHECK_FALSE(run_compute_image("#version 450\nvoid main() { broken }\n", {&src}, dst, {}, &err));
  CHECK_FALSE(err.empty());
}

TEST_CASE("compile_glsl/run_compute: 同一ソースの2回目はキャッシュヒットする") {
  if(!vk_ready()) return;
  const std::string glsl = std::string(kInvert) + "// cache-test-unique\n";
  Image src(8, 8), dst(8, 8);
  fill_random(src, 2);
  const auto c0 = shader_compile_count();
  const auto p0 = compute_pipeline_cache_size();
  REQUIRE(run_compute_image(glsl, {&src}, dst));
  REQUIRE(run_compute_image(glsl, {&src}, dst));
  CHECK(shader_compile_count() - c0 == 1);
  CHECK(compute_pipeline_cache_size() - p0 == 1);
}

TEST_CASE("run_compute_image: push constantとUBOの値が反映される") {
  if(!vk_ready()) return;
  const char* glsl = R"(#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(set=0, binding=0, rgba8) uniform writeonly image2D dst;
layout(set=0, binding=1, std140) uniform U { vec4 color; } u;
layout(push_constant) uniform P { float k; } pc;
void main() {
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(any(greaterThanEqual(p, imageSize(dst)))) return;
  imageStore(dst, p, u.color * pc.k);
}
)";
  Image dst(20, 20);
  const float k        = 0.5f;
  const float color[4] = {1.0f, 0.5f, 0.0f, 1.0f};
  ComputeParams params;
  params.push      = &k;
  params.push_size = sizeof(k);
  params.ubo       = color;
  params.ubo_size  = sizeof(color);
  std::string err;
  REQUIRE_MESSAGE(run_compute_image(glsl, {}, dst, params, &err), err);
  Vec4b px = dst.data()[5 * 20 + 7];
  CHECK(std::abs((int)px[0] - 128) <= 1);
  CHECK(std::abs((int)px[1] - 64) <= 1);
  CHECK(px[2] == 0);
  CHECK(std::abs((int)px[3] - 128) <= 1);
}

TEST_CASE("run_compute: 複数入力・複数出力") {
  if(!vk_ready()) return;
  const char* glsl = R"(#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(set=0, binding=0, rgba8) uniform readonly image2D a;
layout(set=0, binding=1, rgba8) uniform readonly image2D b;
layout(set=0, binding=2, rgba8) uniform writeonly image2D sum;
layout(set=0, binding=3, rgba8) uniform writeonly image2D diff;
void main() {
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(any(greaterThanEqual(p, imageSize(sum)))) return;
  vec4 x = imageLoad(a, p), y = imageLoad(b, p);
  imageStore(sum, p, min(x + y, vec4(1.0)));
  imageStore(diff, p, max(x - y, vec4(0.0)));
}
)";
  const int w = 32, h = 16;
  Image a(w, h), b(w, h), rs, rd;
  fill_random(a, 3);
  fill_random(b, 4);
  GpuImage ga(w, h), gb(w, h), gs(w, h), gd(w, h);
  REQUIRE(ga.upload(a));
  REQUIRE(gb.upload(b));
  std::string err;
  REQUIRE_MESSAGE(run_compute(glsl, {&ga, &gb}, {&gs, &gd}, {}, 2, 1, 1, &err), err);
  REQUIRE(gs.readback(rs));
  REQUIRE(gd.readback(rd));
  auto* pa     = reinterpret_cast<uint8_t*>(a.data());
  auto* pb     = reinterpret_cast<uint8_t*>(b.data());
  auto* ps     = reinterpret_cast<uint8_t*>(rs.data());
  auto* pd     = reinterpret_cast<uint8_t*>(rd.data());
  int max_diff = 0;
  for(size_t i = 0; i < a.size_in_bytes(); i++) {
    max_diff = std::max(max_diff, std::abs(std::min(pa[i] + pb[i], 255) - (int)ps[i]));
    max_diff = std::max(max_diff, std::abs(std::max(pa[i] - pb[i], 0) - (int)pd[i]));
  }
  CHECK(max_diff <= 1);
}

TEST_CASE("make_random_image: 値域[0,1)でseed固定なら再現する") {
  if(!vk_ready()) return;
  auto read = [](uint32_t seed) {
    auto img = make_random_image(seed);
    REQUIRE(img != nullptr);
    CHECK(img->width() == 256);
    CHECK(img->height() == 256);
    std::vector<uint8_t> raw;
    REQUIRE(img->readback_raw(raw));
    std::vector<float> v(raw.size() / 4);
    std::memcpy(v.data(), raw.data(), raw.size());
    return v;
  };
  auto a = read(1), b = read(1), c = read(2);
  for(float x : a) {
    CHECK(x >= 0.0f);
    CHECK(x < 1.0f);
  }
  CHECK(a == b);
  CHECK(a != c);
}
