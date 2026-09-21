#pragma once
#include <cutil/ref.hpp>
#include <imgui.h>
#include <memory>
#include <movutl/asset/image.hpp>
#include <movutl/vulkan/vk_image.hpp>

namespace mu {

// CPUのImageをGPUへ転送してImGuiで表示するためのテクスチャ(旧GLTextureの置き換え)
// ImGui_ImplVulkan_AddTextureのVkDescriptorSetをImTextureIDとして使う。ImGuiのVulkanバックエンド初期化後に使うこと
class VkTexture {
  std::unique_ptr<GpuImage> img_;
  VkSampler sampler_    = VK_NULL_HANDLE;
  VkDescriptorSet desc_ = VK_NULL_HANDLE;
  cutil::WeakPtr<Image> src_;
  uint16_t last_dirty_ = 0;

public:
  VkTexture() = default;
  ~VkTexture() { destroy(); }
  VkTexture(const VkTexture&)            = delete;
  VkTexture& operator=(const VkTexture&) = delete;

  void set(const cutil::Ref<Image>& img);
  void update_if_necessary();
  bool destroy();
  bool initialized() const { return desc_ != VK_NULL_HANDLE; }
  ImTextureID id() const { return (ImTextureID)(uintptr_t)desc_; }
};

} // namespace mu
