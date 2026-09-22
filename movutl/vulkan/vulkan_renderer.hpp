#pragma once

#include <memory>
#include <movutl/asset/entity.hpp>
#include <movutl/render2d/renderer.hpp>
#include <movutl/vulkan/vk_image.hpp>

namespace mu {

// レイヤ合成をGPUで行うRenderer。配置(移動/回転/拡大/alpha/blend)のみGpuCompositeSink経由でGPUへ差し替え、target実内容に依存する経路(Framebuffer/CustomObject/scene_change/clipping_up)はCPUブリッジで橋渡しする。Vulkan不可時はCPURendererへ委譲
class VulkanRenderer : public Renderer, private GpuCompositeSink {
public:
  using Renderer::render_frame;
  VulkanRenderer();
  bool render_frame(Composition* comp, int frame, Ref<Image>& out, bool transparent_bg) override;

private:
  bool place(const Image& src, Image* target, const Placement& pl) override;

  void ensure_target(int w, int h);

  bool ready_ = false;
  CPURenderer fallback_; // Vulkan初期化失敗時、あるいはこのフレームで失敗した場合に使う
  std::unique_ptr<GpuImage> gpu_out_;
};

} // namespace mu
