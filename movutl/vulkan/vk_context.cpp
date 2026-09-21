#include <algorithm>
#include <cstring>
#include <movutl/core/logger.hpp>
#include <movutl/vulkan/vk_context.hpp>
#include <set>

namespace mu {

VkContext* VkContext::singleton_ = nullptr;

VkContext* VkContext::Get() {
  static std::once_flag once;
  std::call_once(once, [] { singleton_ = new VkContext(); });
  return singleton_;
}

namespace {
bool has_ext(const std::vector<VkExtensionProperties>& v, const char* name) {
  return std::any_of(v.begin(), v.end(), [&](const VkExtensionProperties& e) { return std::strcmp(e.extensionName, name) == 0; });
}
} // namespace

bool VkContext::create(const std::vector<std::string>& instance_exts, bool need_swapchain, const PresentCheck& present_check) {
  std::lock_guard<std::mutex> lock(create_mtx_);
  if(valid()) return true;

  uint32_t n = 0;
  vkEnumerateInstanceExtensionProperties(nullptr, &n, nullptr);
  std::vector<VkExtensionProperties> avail_ext(n);
  vkEnumerateInstanceExtensionProperties(nullptr, &n, avail_ext.data());

  std::set<std::string> exts(instance_exts.begin(), instance_exts.end());
  VkInstanceCreateFlags flags = 0;
  // MoltenVK等のportabilityドライバはこの拡張とフラグが無いとloaderに列挙されない
  if(has_ext(avail_ext, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME)) {
    exts.insert(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
  }
  if(has_ext(avail_ext, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME)) exts.insert(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
  std::vector<const char*> ext_ptrs;
  for(auto& e : exts) ext_ptrs.push_back(e.c_str());

  VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
  app.pApplicationName = "movutl";
  app.apiVersion       = VK_API_VERSION_1_1;
  VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
  ici.flags                   = flags;
  ici.pApplicationInfo        = &app;
  ici.enabledExtensionCount   = (uint32_t)ext_ptrs.size();
  ici.ppEnabledExtensionNames = ext_ptrs.data();
  VkInstance instance         = VK_NULL_HANDLE;
  if(vkCreateInstance(&ici, nullptr, &instance) != VK_SUCCESS) {
    LOG_F(WARNING, "vkCreateInstance failed (Vulkanが利用できません)");
    return false;
  }

  uint32_t np = 0;
  vkEnumeratePhysicalDevices(instance, &np, nullptr);
  std::vector<VkPhysicalDevice> phys(np);
  vkEnumeratePhysicalDevices(instance, &np, phys.data());

  // graphics+compute対応のqueue familyを持つデバイスを選ぶ(discrete GPU優先)
  VkPhysicalDevice best = VK_NULL_HANDLE;
  uint32_t best_family  = 0;
  int best_score        = -1;
  for(auto pd : phys) {
    uint32_t nq = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, nullptr);
    std::vector<VkQueueFamilyProperties> qf(nq);
    vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, qf.data());
    if(need_swapchain) {
      uint32_t ne = 0;
      vkEnumerateDeviceExtensionProperties(pd, nullptr, &ne, nullptr);
      std::vector<VkExtensionProperties> de(ne);
      vkEnumerateDeviceExtensionProperties(pd, nullptr, &ne, de.data());
      if(!has_ext(de, VK_KHR_SWAPCHAIN_EXTENSION_NAME)) continue;
    }
    for(uint32_t i = 0; i < nq; i++) {
      if(!(qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) || !(qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT)) continue;
      if(present_check && !present_check(instance, pd, i)) continue;
      VkPhysicalDeviceProperties props;
      vkGetPhysicalDeviceProperties(pd, &props);
      int score = props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 2 : props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 1 : 0;
      if(score > best_score) {
        best_score  = score;
        best        = pd;
        best_family = i;
      }
      break;
    }
  }
  if(best == VK_NULL_HANDLE) {
    LOG_F(WARNING, "利用可能なVulkanデバイスがありません");
    vkDestroyInstance(instance, nullptr);
    return false;
  }

  uint32_t ne = 0;
  vkEnumerateDeviceExtensionProperties(best, nullptr, &ne, nullptr);
  std::vector<VkExtensionProperties> de(ne);
  vkEnumerateDeviceExtensionProperties(best, nullptr, &ne, de.data());
  std::vector<const char*> dev_exts;
  if(need_swapchain) dev_exts.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  if(has_ext(de, "VK_KHR_portability_subset")) dev_exts.push_back("VK_KHR_portability_subset"); // portabilityデバイスでは有効化必須

  float prio = 1.0f;
  VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
  qci.queueFamilyIndex = best_family;
  qci.queueCount       = 1;
  qci.pQueuePriorities = &prio;
  VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
  dci.queueCreateInfoCount    = 1;
  dci.pQueueCreateInfos       = &qci;
  dci.enabledExtensionCount   = (uint32_t)dev_exts.size();
  dci.ppEnabledExtensionNames = dev_exts.data();
  VkDevice device             = VK_NULL_HANDLE;
  if(vkCreateDevice(best, &dci, nullptr, &device) != VK_SUCCESS) {
    LOG_F(WARNING, "vkCreateDevice failed");
    vkDestroyInstance(instance, nullptr);
    return false;
  }

  instance_     = instance;
  phys_         = best;
  queue_family_ = best_family;
  vkGetDeviceQueue(device, best_family, 0, &queue_);
  VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  pci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pci.queueFamilyIndex = best_family;
  vkCreateCommandPool(device, &pci, nullptr, &cmd_pool_);
  VkPhysicalDeviceProperties props;
  vkGetPhysicalDeviceProperties(best, &props);
  device_name_ = props.deviceName;
  device_      = device;
  LOG_F(INFO, "Vulkan device: %s", device_name_.c_str());
  return true;
}

void VkContext::submit_once(const std::function<void(VkCommandBuffer)>& record) {
  std::lock_guard<std::mutex> lock(queue_mtx_);
  VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  ai.commandPool        = cmd_pool_;
  ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  ai.commandBufferCount = 1;
  VkCommandBuffer cb;
  vkAllocateCommandBuffers(device_, &ai, &cb);
  VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cb, &bi);
  record(cb);
  vkEndCommandBuffer(cb);
  VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  si.commandBufferCount = 1;
  si.pCommandBuffers    = &cb;
  vkQueueSubmit(queue_, 1, &si, VK_NULL_HANDLE);
  vkQueueWaitIdle(queue_);
  vkFreeCommandBuffers(device_, cmd_pool_, 1, &cb);
}

void VkContext::wait_idle() {
  std::lock_guard<std::mutex> lock(queue_mtx_);
  vkQueueWaitIdle(queue_);
}

uint32_t VkContext::find_memory_type(uint32_t type_bits, VkMemoryPropertyFlags props) const {
  VkPhysicalDeviceMemoryProperties mp;
  vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
  for(uint32_t i = 0; i < mp.memoryTypeCount; i++)
    if((type_bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & props) == props) return i;
  return UINT32_MAX;
}

VkHostBuffer::VkHostBuffer(size_t sz, VkBufferUsageFlags usage) : size(sz) {
  auto* c = VkContext::Get();
  auto d  = c->device();
  VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  bi.size        = std::max<size_t>(sz, 4);
  bi.usage       = usage;
  bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  vkCreateBuffer(d, &bi, nullptr, &buffer);
  VkMemoryRequirements req;
  vkGetBufferMemoryRequirements(d, buffer, &req);
  VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
  mai.allocationSize  = req.size;
  mai.memoryTypeIndex = c->find_memory_type(req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  vkAllocateMemory(d, &mai, nullptr, &memory);
  vkBindBufferMemory(d, buffer, memory, 0);
  vkMapMemory(d, memory, 0, VK_WHOLE_SIZE, 0, &mapped);
}

VkHostBuffer::~VkHostBuffer() {
  auto d = VkContext::Get()->device();
  if(mapped) vkUnmapMemory(d, memory);
  if(buffer) vkDestroyBuffer(d, buffer, nullptr);
  if(memory) vkFreeMemory(d, memory, nullptr);
}

} // namespace mu
