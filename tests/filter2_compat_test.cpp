#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/gpu/filter2_compat.hpp>
#include <movutl/plugin/gpu/gpu_effects.hpp>
#include <movutl/vulkan/vk_context.hpp>

using namespace mu;
using namespace mu::detail;

namespace {
bool vk_ready() {
  if(VkContext::Get()->create()) return true;
  MESSAGE("SKIP: Vulkanデバイスが利用できません");
  return false;
}
} // namespace

TEST_CASE("filter2_compat: exec_pixelshader_dataでobjectに単色出力") {
  if(!vk_ready()) return;
  Image img(8, 8);
  img.fill_rgba(Vec4b(0, 0, 0, 0));
  Filter2Context ctx(img);

  const char* glsl = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform writeonly image2D dst;
void main() {
  ivec2 sz = imageSize(dst);
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  imageStore(dst, p, vec4(0.2, 0.4, 0.6, 1.0));
}
)GLSL";
  REQUIRE(ctx.exec_pixelshader_data(glsl, (int)strlen(glsl), nullptr, nullptr, 0, nullptr, 0));
  ctx.sync_object_to_image();
  CHECK(img.data()[0][0] == 51);  // 0.2*255
  CHECK(img.data()[0][1] == 102); // 0.4*255
  CHECK(img.data()[0][2] == 153); // 0.6*255
}

TEST_CASE("filter2_compat: exec_computeshaderでobjectを反転する(gpu_invertと一致)") {
  if(!vk_ready()) return;
  Image img_a(6, 5), img_b(6, 5);
  for(size_t i = 0; i < img_a.size(); i++) img_a.data()[i] = img_b.data()[i] = Vec4b((uint8_t)(i * 7), (uint8_t)(i * 13), (uint8_t)(i * 3), 255);

  Filter2Context ctx(img_a);
  const char* glsl         = R"GLSL(
#version 450
layout(local_size_x=16, local_size_y=16) in;
layout(binding=0, rgba8) uniform image2D dst;
void main() {
  ivec2 sz = imageSize(dst);
  ivec2 p = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  vec4 c = imageLoad(dst, p);
  c.rgb = vec3(1.0) - c.rgb;
  imageStore(dst, p, c);
}
)GLSL";
  const wchar_t* targets[] = {L"object"};
  REQUIRE(ctx.exec_computeshader_data(glsl, (int)strlen(glsl), targets, 1, nullptr, 0, nullptr, 0, 1, 1, 1));
  ctx.sync_object_to_image();

  REQUIRE(gpu_invert(img_b, false));
  for(size_t i = 0; i < img_a.size(); i++)
    for(int c = 0; c < 3; c++) CHECK(std::abs((int)img_a.data()[i][c] - (int)img_b.data()[i][c]) <= 1);
}

TEST_CASE("filter2_compat: draw_polyでTRIANGLE_COLORの三角形が描ける") {
  if(!vk_ready()) return;
  Image img(20, 20);
  img.fill_rgba(Vec4b(0, 0, 0, 255));
  Filter2Context ctx(img);

  // 中心付近を覆う大きな三角形(dst中心原点・Y下向き座標系)
  VERTEX_COLOR tri[3] = {
    {0, -8, 0, 1, 0, 0, 1},
    {-8, 8, 0, 1, 0, 0, 1},
    {8, 8, 0, 1, 0, 0, 1},
  };
  REQUIRE(ctx.draw_poly(VERTEX_TYPE::TRIANGLE_COLOR, tri, 3, nullptr));
  ctx.sync_object_to_image();
  CHECK(img.data()[10 * 20 + 10] == Vec4b(255, 0, 0, 255)); // 中心(10,10)は三角形内
  CHECK(img.data()[0] == Vec4b(0, 0, 0, 255));              // 左上角は三角形外のまま
}

TEST_CASE("filter2_compat: draw_polyでQUAD_TEXTUREの四角形が描ける(objectを自己テクスチャとして塗る)") {
  if(!vk_ready()) return;
  Image img(20, 20);
  for(size_t i = 0; i < img.size(); i++) img.data()[i] = Vec4b(10, 20, 30, 255);
  Filter2Context ctx(img);

  VERTEX_TEXTURE quad[4] = {
    {-8, -8, 0, 0, 0, 1},
    {8, -8, 0, 1, 0, 1},
    {8, 8, 0, 1, 1, 1},
    {-8, 8, 0, 0, 1, 1},
  };
  REQUIRE(ctx.draw_poly(VERTEX_TYPE::QUAD_TEXTURE, quad, 4, nullptr)); // resource省略時はobject自身をテクスチャに使う
  ctx.sync_object_to_image();
  CHECK(img.data()[10 * 20 + 10] == Vec4b(10, 20, 30, 255)); // 中心は四角形内、テクスチャはobject自身なので変化なし
}

TEST_CASE("filter2_compat: 不正な頂点数・リソース名でfalse") {
  if(!vk_ready()) return;
  Image img(10, 10);
  Filter2Context ctx(img);
  VERTEX_COLOR tri[2] = {{0, 0, 0, 1, 0, 0, 1}, {1, 1, 0, 1, 0, 0, 1}};
  CHECK_FALSE(ctx.draw_poly(VERTEX_TYPE::TRIANGLE_COLOR, tri, 2, nullptr)); // 3の倍数でない
  CHECK_FALSE(ctx.draw_poly(VERTEX_TYPE::TRIANGLE_COLOR, nullptr, 3, nullptr));
  const wchar_t* bad_target[] = {L"image:/no/such/file.png"};
  CHECK_FALSE(ctx.exec_computeshader_data("x", 1, bad_target, 1, nullptr, 0, nullptr, 0, 1, 1, 1));
}

TEST_CASE("filter2_compat: .cso指定でfalse") {
  if(!vk_ready()) return;
  Image img(4, 4);
  Filter2Context ctx(img);
  CHECK_FALSE(ctx.exec_pixelshader_file(L"shader.cso", nullptr, nullptr, 0, nullptr, 0));
  CHECK_FALSE(ctx.exec_computeshader_file(L"shader.cso", nullptr, 0, nullptr, 0, nullptr, 0, 1, 1, 1));
}

TEST_CASE("filter2_compat: set_blend_modeがdraw_imageに反映される(ADD)") {
  if(!vk_ready()) return;
  // "object"自身をsrcとして自己合成し、200+200=400をclampした255になることでADD合成を確認する
  Image dst(4, 4);
  dst.fill_rgba(Vec4b(200, 200, 200, 255));
  Filter2Context ctx(dst);
  ctx.set_blend_mode(BLEND_MODE::ADD);
  REQUIRE(ctx.draw_image(nullptr, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1.0f));
  ctx.sync_object_to_image();
  CHECK(dst.data()[0][0] == 255);
}
