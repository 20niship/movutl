#include <imgui_impl_vulkan.h>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/graphics/VkTexture.hpp>

namespace mu {

void VkTexture::set(const cutil::Ref<Image>& image) {
  MOVUTL_ZONE_SCOPED_N("VkTexture::set");
  MU_ASSERT(image);
  if(image->width == 0 || image->height == 0) {
    LOG_F(ERROR, "Image size is zero. s (%d, %d)", image->width, image->height);
    return;
  }
  auto* ctx = VkContext::Get();
  if(!ctx->valid()) return;
  src_ = image;
  // 直前のフレームがこのテクスチャを参照している可能性があるため、作り直す前にGPUの完了を待つ
  if(!img_ || img_->width() != image->width || img_->height() != image->height) {
    destroy();
    img_ = std::make_unique<GpuImage>(image->width, image->height);
    if(!img_->valid()) {
      img_.reset();
      return;
    }
    img_->set_rest_layout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    si.magFilter    = VK_FILTER_LINEAR;
    si.minFilter    = VK_FILTER_LINEAR;
    si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    vkCreateSampler(ctx->device(), &si, nullptr, &sampler_);
  } else {
    ctx->wait_idle();
  }
  last_dirty_ = image->dirty_;
  img_->upload(*image);
  if(!desc_) desc_ = ImGui_ImplVulkan_AddTexture(sampler_, img_->view(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

void VkTexture::update_if_necessary() {
  MOVUTL_ZONE_SCOPED_N("VkTexture::update_if_necessary");
  if(src_.expired() || !img_) return;
  auto src = src_.lock();
  if(src->dirty_ == last_dirty_) return;
  set(src);
}

bool VkTexture::destroy() {
  if(!img_ && !desc_) return false;
  auto* ctx = VkContext::Get();
  ctx->wait_idle();
  if(desc_) ImGui_ImplVulkan_RemoveTexture(desc_);
  if(sampler_) vkDestroySampler(ctx->device(), sampler_, nullptr);
  desc_    = VK_NULL_HANDLE;
  sampler_ = VK_NULL_HANDLE;
  img_.reset();
  return true;
}

} // namespace mu
