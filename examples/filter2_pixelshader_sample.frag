// filter2互換API(movutl/plugin/gpu/filter2_compat.hpp)のexec_pixelshader_file向けサンプル。
// Filter2Context::exec_pixelshader_file(L"filter2_pixelshader_sample.frag", L"object", nullptr, 0, nullptr, 0) のように呼ぶ。
// 拡張子は.fragだが実体はVulkan compute shader(movutl/vulkan/gpu_compute.hppのGLSL規約)。
// binding=0にresource_listの入力、続けてtargetの出力がstorage imageとして並ぶ(このサンプルは出力のみ)。
#version 450
layout(local_size_x = 16, local_size_y = 16) in;
layout(binding = 0, rgba8) uniform writeonly image2D dst;

void main() {
  ivec2 sz = imageSize(dst);
  ivec2 p  = ivec2(gl_GlobalInvocationID.xy);
  if(p.x >= sz.x || p.y >= sz.y) return;
  vec2 uv = (vec2(p) + 0.5) / vec2(sz);
  imageStore(dst, p, vec4(uv, 0.5, 1.0));
}
