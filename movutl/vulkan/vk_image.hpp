#pragma once
#include <cstdint>
#include <movutl/asset/image.hpp>
#include <movutl/vulkan/vk_context.hpp>
#include <vector>

namespace mu {

// GPU上の2D画像(device local)。1画素4バイトのフォーマット(RGBA8/BGRA8/R32F等)のみ扱う
// upload/readbackは同期実行(VkContext::submit_once)。完了後は rest_layout(既定GENERAL)に戻る
class GpuImage {
public:
  static constexpr VkImageUsageFlags kDefaultUsage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

  GpuImage(uint32_t w, uint32_t h, VkFormat fmt = VK_FORMAT_R8G8B8A8_UNORM, VkImageUsageFlags usage = kDefaultUsage);
  ~GpuImage();
  GpuImage(const GpuImage&)            = delete;
  GpuImage& operator=(const GpuImage&) = delete;

  bool valid() const { return image_ != VK_NULL_HANDLE; }
  uint32_t width() const { return w_; }
  uint32_t height() const { return h_; }
  VkFormat format() const { return fmt_; }
  VkImage image() const { return image_; }
  VkImageView view() const { return view_; }
  VkImageLayout layout() const { return layout_; }
  void set_rest_layout(VkImageLayout l) { rest_layout_ = l; }

  // RGBA8のImageを転送する。sizeが一致しない場合はfalse
  bool upload(Image& img);
  // outをwidth x heightにresizeして書き込む(RGBA8のみ)
  bool readback(Image& out);
  bool upload_raw(const void* data, size_t bytes);
  bool readback_raw(std::vector<uint8_t>& out);

  // 既存のコマンドバッファ内でlayout遷移を記録する
  void transition(VkCommandBuffer cb, VkImageLayout new_layout);

private:
  uint32_t w_ = 0, h_ = 0;
  VkFormat fmt_              = VK_FORMAT_UNDEFINED;
  VkImage image_             = VK_NULL_HANDLE;
  VkDeviceMemory memory_     = VK_NULL_HANDLE;
  VkImageView view_          = VK_NULL_HANDLE;
  VkImageLayout layout_      = VK_IMAGE_LAYOUT_UNDEFINED;
  VkImageLayout rest_layout_ = VK_IMAGE_LAYOUT_GENERAL;
};

} // namespace mu
