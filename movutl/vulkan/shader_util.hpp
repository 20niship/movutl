#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace mu {

enum class ShaderStage { Compute, Fragment, Vertex };

struct ShaderCompileResult {
  bool ok = false;
  std::vector<uint32_t> spirv;
  std::string error; // glslangのログ("ERROR: 0:<行>: ..."形式で行番号を含む)
};

// GLSL(#version 450 等)をVulkan向けSPIR-Vへコンパイルする。成功結果はソース文字列をキーにキャッシュされる(失敗は毎回再コンパイル)
// スレッドセーフ
ShaderCompileResult compile_glsl(const std::string& glsl, ShaderStage stage = ShaderStage::Compute);

// 実際にコンパイルした回数(キャッシュヒットは数えない)。テストでキャッシュ確認に使う
uint64_t shader_compile_count();

} // namespace mu
