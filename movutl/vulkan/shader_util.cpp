#include <atomic>
#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <glslang/SPIRV/GlslangToSpv.h>
#include <movutl/vulkan/shader_util.hpp>
#include <mutex>
#include <unordered_map>

namespace mu {

namespace {
std::mutex g_mtx;
std::unordered_map<std::string, std::vector<uint32_t>> g_cache;
std::atomic<uint64_t> g_count{0};

EShLanguage to_lang(ShaderStage s) {
  switch(s) {
    case ShaderStage::Fragment: return EShLangFragment;
    case ShaderStage::Vertex: return EShLangVertex;
    default: return EShLangCompute;
  }
}
} // namespace

ShaderCompileResult compile_glsl(const std::string& glsl, ShaderStage stage) {
  // glslangのinitializeはプロセス内で1回。TShader/TProgramの生成・解析もグローバル状態を触るため同じmutexで直列化する
  std::lock_guard<std::mutex> lock(g_mtx);
  static bool inited = (glslang::InitializeProcess(), true);
  (void)inited;

  ShaderCompileResult res;
  const std::string key = std::to_string((int)stage) + ":" + glsl;
  if(auto it = g_cache.find(key); it != g_cache.end()) {
    res.ok    = true;
    res.spirv = it->second;
    return res;
  }

  g_count++;
  glslang::TShader shader(to_lang(stage));
  const char* src = glsl.c_str();
  shader.setStrings(&src, 1);
  shader.setEnvInput(glslang::EShSourceGlsl, to_lang(stage), glslang::EShClientVulkan, 100);
  shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_1);
  shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_3);
  const EShMessages msgs = (EShMessages)(EShMsgSpvRules | EShMsgVulkanRules);
  if(!shader.parse(GetDefaultResources(), 450, false, msgs)) {
    res.error = std::string("GLSL compile error:\n") + shader.getInfoLog();
    return res;
  }
  glslang::TProgram program;
  program.addShader(&shader);
  if(!program.link(msgs)) {
    res.error = std::string("GLSL link error:\n") + program.getInfoLog();
    return res;
  }
  glslang::GlslangToSpv(*program.getIntermediate(to_lang(stage)), res.spirv);
  res.ok       = true;
  g_cache[key] = res.spirv;
  return res;
}

uint64_t shader_compile_count() { return g_count.load(); }

} // namespace mu
