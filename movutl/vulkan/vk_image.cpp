#include <cstring>
#include <movutl/core/logger.hpp>
#include <movutl/vulkan/vk_image.hpp>

namespace mu {

GpuImage::GpuImage(uint32_t w, uint32_t h, VkFormat fmt, VkImageUsageFlags usage) : w_(w), h_(h), fmt_(fmt) {
  auto* c = VkContext::Get();
  if(!c->valid() || w == 0 || h == 0) return;
  auto d = c->device();
  VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  ii.imageType     = VK_IMAGE_TYPE_2D;
  ii.format        = fmt;
  ii.extent        = {w, h, 1};
  ii.mipLevels     = 1;
  ii.arrayLayers   = 1;
  ii.samples       = VK_SAMPLE_COUNT_1_BIT;
  ii.tiling        = VK_IMAGE_TILING_OPTIMAL;
  ii.usage         = usage;
  ii.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
  ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  if(vkCreateImage(d, &ii, nullptr, &image_) != VK_SUCCESS) {
    image_ = VK_NULL_HANDLE;
    LOG_F(ERROR, "vkCreateImage failed (%u x %u)", w, h);
    return;
  }
  VkMemoryRequirements req;
  vkGetImageMemoryRequirements(d, image_, &req);
  VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
  mai.allocationSize  = req.size;
  mai.memoryTypeIndex = c->find_memory_type(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if(vkAllocateMemory(d, &mai, nullptr, &memory_) != VK_SUCCESS) {
    vkDestroyImage(d, image_, nullptr);
    image_  = VK_NULL_HANDLE;
    memory_ = VK_NULL_HANDLE;
    LOG_F(ERROR, "vkAllocateMemory failed for GpuImage");
    return;
  }
  vkBindImageMemory(d, image_, memory_, 0);
  VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
  vi.image                       = image_;
  vi.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
  vi.format                      = fmt;
  vi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  vi.subresourceRange.levelCount = 1;
  vi.subresourceRange.layerCount = 1;
  vkCreateImageView(d, &vi, nullptr, &view_);
}

GpuImage::~GpuImage() {
  if(!image_) return;
  auto d = VkContext::Get()->device();
  vkDestroyImageView(d, view_, nullptr);
  vkDestroyImage(d, image_, nullptr);
  vkFreeMemory(d, memory_, nullptr);
}

void GpuImage::transition(VkCommandBuffer cb, VkImageLayout nl) {
  if(layout_ == nl) return;
  VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
  b.oldLayout                   = layout_;
  b.newLayout                   = nl;
  b.srcQueueFamilyIndex         = VK_QUEUE_FAMILY_IGNORED;
  b.dstQueueFamilyIndex         = VK_QUEUE_FAMILY_IGNORED;
  b.image                       = image_;
  b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  b.subresourceRange.levelCount = 1;
  b.subresourceRange.layerCount = 1;
  b.srcAccessMask               = VK_ACCESS_MEMORY_WRITE_BIT;
  b.dstAccessMask               = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
  // ponytail: 全stage待ちの粗いbarrier。パイプラインが増えて計測で効いてきたら必要stageに絞る
  vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
  layout_ = nl;
}

bool GpuImage::upload_raw(const void* data, size_t bytes) {
  if(!valid() || bytes != (size_t)w_ * h_ * 4) return false;
  VkHostBuffer stage(bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT); // ponytail: 毎回staging確保。頻繁に呼ぶ経路(Phase4)ではプールする
  std::memcpy(stage.mapped, data, bytes);
  VkContext::Get()->submit_once([&](VkCommandBuffer cb) {
    transition(cb, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    VkBufferImageCopy r{};
    r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    r.imageExtent      = {w_, h_, 1};
    vkCmdCopyBufferToImage(cb, stage.buffer, image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r);
    transition(cb, rest_layout_);
  });
  return true;
}

std::atomic<uint64_t>& gpu_readback_count() {
  static std::atomic<uint64_t> n{0};
  return n;
}

bool GpuImage::readback_raw(std::vector<uint8_t>& out) {
  if(!valid()) return false;
  gpu_readback_count()++;
  const size_t bytes = (size_t)w_ * h_ * 4;
  VkHostBuffer stage(bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  VkContext::Get()->submit_once([&](VkCommandBuffer cb) {
    transition(cb, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    VkBufferImageCopy r{};
    r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    r.imageExtent      = {w_, h_, 1};
    vkCmdCopyImageToBuffer(cb, image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, stage.buffer, 1, &r);
    transition(cb, rest_layout_);
  });
  out.resize(bytes);
  std::memcpy(out.data(), stage.mapped, bytes);
  return true;
}

bool GpuImage::upload(Image& img) {
  if(img.width != w_ || img.height != h_) return false;
  return upload_raw(img.data(), img.size_in_bytes());
}

bool GpuImage::readback(Image& out) {
  if(!valid() || fmt_ != VK_FORMAT_R8G8B8A8_UNORM) return false;
  std::vector<uint8_t> buf;
  if(!readback_raw(buf)) return false;
  out.resize((int)w_, (int)h_);
  std::memcpy(out.data(), buf.data(), buf.size());
  out.dirty();
  return true;
}

} // namespace mu
