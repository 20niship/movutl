#pragma once
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>

namespace mu {

// Vulkanのinstance/device/queueを1つだけ持つ共有コンテキスト。GUI(swapchain)とレンダラー/computeで同じVkDeviceを使う
// ヘッドレス(テスト)ではcreate({}, false)、GUIではGLFWが要求するinstance拡張とpresent対応判定を渡してcreateする
class VkContext {
public:
  static VkContext* Get();

  using PresentCheck = std::function<bool(VkInstance, VkPhysicalDevice, uint32_t queue_family)>;
  // 生成済みならそのままtrue。デバイスが無い等で失敗したらfalseを返し(ログ出力)、以降も未生成のまま
  bool create(const std::vector<std::string>& instance_exts = {}, bool need_swapchain = false, const PresentCheck& present_check = nullptr);
  bool valid() const { return device_ != VK_NULL_HANDLE; }

  VkInstance instance() const { return instance_; }
  VkPhysicalDevice physical_device() const { return phys_; }
  VkDevice device() const { return device_; }
  VkQueue queue() const { return queue_; }
  uint32_t queue_family() const { return queue_family_; }
  const char* device_name() const { return device_name_.c_str(); }

  // コマンドバッファを1本記録してsubmitし完了まで待つ(同期実行)。queueはmutexで直列化される
  // ponytail: 毎回vkQueueWaitIdleで待つ簡易実装。レンダーワーカー並列化(Phase4)でスレッド毎command pool+fenceに置き換える
  void submit_once(const std::function<void(VkCommandBuffer)>& record);
  void wait_idle();
  // GUIのフレーム描画submitなど、呼び出し側でqueueを直接使う場合に取るロック
  std::mutex& queue_mutex() { return queue_mtx_; }

  uint32_t find_memory_type(uint32_t type_bits, VkMemoryPropertyFlags props) const;

private:
  VkContext()             = default;
  VkInstance instance_    = VK_NULL_HANDLE;
  VkPhysicalDevice phys_  = VK_NULL_HANDLE;
  VkDevice device_        = VK_NULL_HANDLE;
  VkQueue queue_          = VK_NULL_HANDLE;
  VkCommandPool cmd_pool_ = VK_NULL_HANDLE;
  uint32_t queue_family_  = 0;
  std::string device_name_;
  std::mutex queue_mtx_;
  std::mutex create_mtx_;
  static VkContext* singleton_;
};

// バッファ(host visible/coherent、常時map)。upload/readbackのstagingとUBOで使う
struct VkHostBuffer {
  VkBuffer buffer       = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  void* mapped          = nullptr;
  size_t size           = 0;
  VkHostBuffer()        = default;
  VkHostBuffer(size_t size, VkBufferUsageFlags usage);
  VkHostBuffer(const VkHostBuffer&)            = delete;
  VkHostBuffer& operator=(const VkHostBuffer&) = delete;
  ~VkHostBuffer();
};

} // namespace mu
