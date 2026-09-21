#include <cstring>
#include <movutl/core/logger.hpp>
#include <movutl/vulkan/gpu_compute.hpp>
#include <movutl/vulkan/shader_util.hpp>
#include <mutex>
#include <random>
#include <unordered_map>

namespace mu {

namespace {
struct Pipeline {
  VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
  VkPipelineLayout layout          = VK_NULL_HANDLE;
  VkPipeline pipeline              = VK_NULL_HANDLE;
};

std::mutex g_mtx;
std::unordered_map<std::string, Pipeline> g_pipelines;
VkDescriptorPool g_pool = VK_NULL_HANDLE;

bool fail(std::string* err, const std::string& msg) {
  if(err) *err = msg;
  LOG_F(WARNING, "%s", msg.c_str());
  return false;
}

VkDescriptorPool pool(VkDevice d) {
  if(g_pool) return g_pool;
  const VkDescriptorPoolSize sizes[] = {{VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 64}, {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 16}};
  VkDescriptorPoolCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
  ci.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  ci.maxSets       = 16;
  ci.poolSizeCount = 2;
  ci.pPoolSizes    = sizes;
  vkCreateDescriptorPool(d, &ci, nullptr, &g_pool);
  return g_pool;
}

// 呼び出し側でg_mtxを取っていること
bool get_pipeline(VkDevice d, const std::string& glsl, uint32_t n_images, bool has_ubo, uint32_t push_size, Pipeline& out, std::string* err) {
  const std::string key = std::to_string(n_images) + "/" + (has_ubo ? "u" : "-") + "/" + std::to_string(push_size) + "/" + glsl;
  if(auto it = g_pipelines.find(key); it != g_pipelines.end()) {
    out = it->second;
    return true;
  }
  auto sp = compile_glsl(glsl, ShaderStage::Compute);
  if(!sp.ok) return fail(err, sp.error);

  std::vector<VkDescriptorSetLayoutBinding> b;
  for(uint32_t i = 0; i < n_images; i++) b.push_back({i, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr});
  if(has_ubo) b.push_back({n_images, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr});
  VkDescriptorSetLayoutCreateInfo lci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
  lci.bindingCount = (uint32_t)b.size();
  lci.pBindings    = b.data();
  Pipeline p;
  vkCreateDescriptorSetLayout(d, &lci, nullptr, &p.set_layout);
  VkPushConstantRange pcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size};
  VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  plci.setLayoutCount         = 1;
  plci.pSetLayouts            = &p.set_layout;
  plci.pushConstantRangeCount = push_size ? 1 : 0;
  plci.pPushConstantRanges    = &pcr;
  vkCreatePipelineLayout(d, &plci, nullptr, &p.layout);

  VkShaderModuleCreateInfo smi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  smi.codeSize = sp.spirv.size() * 4;
  smi.pCode    = sp.spirv.data();
  VkShaderModule sm;
  vkCreateShaderModule(d, &smi, nullptr, &sm);
  VkComputePipelineCreateInfo pci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
  pci.stage  = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, sm, "main", nullptr};
  pci.layout = p.layout;
  VkResult r = vkCreateComputePipelines(d, VK_NULL_HANDLE, 1, &pci, nullptr, &p.pipeline);
  vkDestroyShaderModule(d, sm, nullptr);
  if(r != VK_SUCCESS) {
    vkDestroyPipelineLayout(d, p.layout, nullptr);
    vkDestroyDescriptorSetLayout(d, p.set_layout, nullptr);
    return fail(err, "vkCreateComputePipelines failed (画像数/push constantサイズがGLSLと合っているか確認してください)");
  }
  g_pipelines[key] = p;
  out              = p;
  return true;
}
} // namespace

bool run_compute(const std::string& glsl, const std::vector<GpuImage*>& inputs, const std::vector<GpuImage*>& outputs, const ComputeParams& params, uint32_t gx, uint32_t gy, uint32_t gz, std::string* err) {
  auto* ctx = VkContext::Get();
  if(!ctx->valid()) return fail(err, "Vulkanが初期化されていません");
  if(params.push_size > 128) return fail(err, "push constantは128バイトまでです");
  std::vector<GpuImage*> imgs = inputs;
  imgs.insert(imgs.end(), outputs.begin(), outputs.end());
  for(auto* i : imgs)
    if(!i || !i->valid()) return fail(err, "無効なGpuImageが渡されました");
  auto d = ctx->device();

  std::lock_guard<std::mutex> lock(g_mtx);
  Pipeline p;
  if(!get_pipeline(d, glsl, (uint32_t)imgs.size(), params.ubo_size > 0, params.push_size, p, err)) return false;

  VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
  ai.descriptorPool     = pool(d);
  ai.descriptorSetCount = 1;
  ai.pSetLayouts        = &p.set_layout;
  VkDescriptorSet set;
  if(vkAllocateDescriptorSets(d, &ai, &set) != VK_SUCCESS) return fail(err, "descriptor set確保に失敗しました");

  std::vector<VkDescriptorImageInfo> ii(imgs.size());
  std::vector<VkWriteDescriptorSet> w;
  for(size_t i = 0; i < imgs.size(); i++) {
    ii[i] = {VK_NULL_HANDLE, imgs[i]->view(), VK_IMAGE_LAYOUT_GENERAL};
    VkWriteDescriptorSet ws{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    ws.dstSet          = set;
    ws.dstBinding      = (uint32_t)i;
    ws.descriptorCount = 1;
    ws.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    ws.pImageInfo      = &ii[i];
    w.push_back(ws);
  }
  // ponytail: UBOは呼び出し毎に小さなbufferを作る。毎フレーム多数呼ぶ用途(Phase6)ではリングバッファ化する
  std::unique_ptr<VkHostBuffer> ubo;
  VkDescriptorBufferInfo bi{};
  if(params.ubo_size > 0) {
    ubo = std::make_unique<VkHostBuffer>(params.ubo_size, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    std::memcpy(ubo->mapped, params.ubo, params.ubo_size);
    bi = {ubo->buffer, 0, params.ubo_size};
    VkWriteDescriptorSet ws{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    ws.dstSet          = set;
    ws.dstBinding      = (uint32_t)imgs.size();
    ws.descriptorCount = 1;
    ws.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    ws.pBufferInfo     = &bi;
    w.push_back(ws);
  }
  vkUpdateDescriptorSets(d, (uint32_t)w.size(), w.data(), 0, nullptr);

  ctx->submit_once([&](VkCommandBuffer cb) {
    for(auto* i : imgs) i->transition(cb, VK_IMAGE_LAYOUT_GENERAL);
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, p.pipeline);
    vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, p.layout, 0, 1, &set, 0, nullptr);
    if(params.push_size) vkCmdPushConstants(cb, p.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, params.push_size, params.push);
    vkCmdDispatch(cb, gx, gy, gz);
    VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT};
    vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &mb, 0, nullptr, 0, nullptr);
  });
  vkFreeDescriptorSets(d, g_pool, 1, &set);
  return true;
}

bool run_compute_image(const std::string& glsl, const std::vector<Image*>& inputs, Image& output, const ComputeParams& params, std::string* err) {
  if(!VkContext::Get()->valid()) return fail(err, "Vulkanが初期化されていません");
  std::vector<std::unique_ptr<GpuImage>> owned;
  std::vector<GpuImage*> in;
  for(auto* img : inputs) {
    owned.push_back(std::make_unique<GpuImage>(img->width, img->height));
    if(!owned.back()->upload(*img)) return fail(err, "入力画像のuploadに失敗しました");
    in.push_back(owned.back().get());
  }
  GpuImage out(output.width, output.height);
  if(!run_compute(glsl, in, {&out}, params, (output.width + 15) / 16, (output.height + 15) / 16, 1, err)) return false;
  return out.readback(output);
}

std::unique_ptr<GpuImage> make_random_image(uint32_t seed) {
  constexpr uint32_t kSize = 256;
  std::mt19937 rng(seed);
  std::vector<float> v(kSize * kSize);
  for(auto& x : v) x = (float)(rng() >> 8) / 16777216.0f; // 24bitを[0,1)へ。distributionは実装依存なので使わない
  auto img = std::make_unique<GpuImage>(kSize, kSize, VK_FORMAT_R32_SFLOAT);
  if(!img->valid() || !img->upload_raw(v.data(), v.size() * sizeof(float))) return nullptr;
  return img;
}

size_t compute_pipeline_cache_size() {
  std::lock_guard<std::mutex> lock(g_mtx);
  return g_pipelines.size();
}

} // namespace mu
