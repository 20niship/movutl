#pragma once
#include <memory>
#include <movutl/asset/image.hpp>
#include <movutl/vulkan/vk_image.hpp>
#include <string>
#include <vector>

namespace mu {

// 汎用compute実行ユーティリティ。エフェクトはVulkan APIを直接触らずこれだけを使う
//
// GLSLの規約(#version 450):
//   - 画像は set=0, binding=0.. に inputs, outputs の順で全て storage image として並べる
//     例) layout(set=0, binding=0, rgba8) uniform readonly image2D src;  layout(set=0, binding=1, rgba8) uniform writeonly image2D dst;
//     R32Fの乱数画像などは layout(r32f) を指定する(GpuImageのformatと一致させる)
//   - ubo を渡す場合は binding=(画像の数) に layout(set=0, binding=N, std140) uniform U { ... };
//   - push constantは layout(push_constant) uniform P { ... }; (size <= 128)
struct ComputeParams {
  const void* push   = nullptr;
  uint32_t push_size = 0;
  const void* ubo    = nullptr;
  uint32_t ubo_size  = 0;
};

// groups(x,y,z)はdispatchのworkgroup数。失敗時はfalseを返しerrにコンパイル/実行エラーを入れる
bool run_compute(const std::string& glsl, const std::vector<GpuImage*>& inputs, const std::vector<GpuImage*>& outputs, const ComputeParams& params, uint32_t gx, uint32_t gy, uint32_t gz = 1, std::string* err = nullptr);

// Imageを直接受ける簡易版(upload→実行→readback)。output(サイズ設定済み)の全画素を対象に
// layout(local_size_x=16, local_size_y=16) in; として ceil(w/16) x ceil(h/16) をdispatchする。画像は inputs..., output の順にbindingされる
bool run_compute_image(const std::string& glsl, const std::vector<Image*>& inputs, Image& output, const ComputeParams& params = {}, std::string* err = nullptr);

// 256x256 R32Fの乱数画像(値域[0,1)、seed固定で同じ内容)。filter2.hの"random"リソース用
std::unique_ptr<GpuImage> make_random_image(uint32_t seed = 1);

// pipeline/descriptor layoutのキャッシュに載っている件数(テスト用)
size_t compute_pipeline_cache_size();

} // namespace mu
