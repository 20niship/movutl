#include <atomic>
#include <condition_variable>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/vst/vst_host.hpp>
#include <mutex>
#include <thread>
#include <uapmd-plugin-hosting/uapmd-plugin-hosting.hpp>

namespace mu::vst_host {
// ponytail: AudioPluginHostingAPIはOS標準VST3パスのみ検索しCustom pathを追加するAPIが無いため、Config::vst_plugin_dirsは未使用。要拡張ならformat層のaddSearchPathを露出。

namespace {
uapmd_plugin_hosting::AudioPluginHostingAPI* api() {
  static std::unique_ptr<uapmd_plugin_hosting::AudioPluginHostingAPI> instance = uapmd_plugin_hosting::AudioPluginHostingAPI::create();
  return instance.get();
}
std::atomic<bool> scanning_{false};
} // namespace

void scan_and_load(std::function<void()> on_complete) {
  if(scanning_.exchange(true)) return;
  std::thread([on_complete = std::move(on_complete)] {
    api()->performPluginScanning(false);
    scanning_.store(false);
    LOG_F(INFO, "vst_host: plugin scan complete (%zu entries)", api()->pluginCatalogEntries().size());
    if(on_complete) on_complete();
  }).detach();
}

bool is_scanning() { return scanning_.load(); }

std::vector<PluginInfo> plugin_list() {
  std::vector<PluginInfo> out;
  for(auto& e : api()->pluginCatalogEntries()) {
    if(e.format() != "VST3") continue; // 方針確定: VST3のみ対象
    out.push_back(PluginInfo{e.pluginId(), e.format(), e.displayName()});
  }
  return out;
}

int32_t create_instance(const std::string& plugin_id, uint32_t sample_rate, uint32_t buffer_size) {
  std::mutex done_mtx;
  std::condition_variable cv;
  bool done      = false;
  int32_t result = -1;
  std::string format{"VST3"};
  std::string id = plugin_id;
  api()->createPluginInstance(sample_rate, buffer_size, 2u, 2u, false, format, id, [&](int32_t instanceId, std::string error) {
    std::lock_guard<std::mutex> lock(done_mtx);
    if(instanceId >= 0) {
      result = instanceId;
    } else {
      LOG_F(ERROR, "vst_host::create_instance failed for %s: %s", plugin_id.c_str(), error.c_str());
    }
    done = true;
    cv.notify_all();
  });
  std::unique_lock<std::mutex> lock(done_mtx);
  cv.wait(lock, [&] { return done; });
  return result;
}

uapmd_plugin_hosting::AudioPluginInstanceAPI* get_instance(int32_t instance_id) {
  if(instance_id < 0) return nullptr;
  return api()->getInstance(instance_id);
}

void destroy_instance(int32_t instance_id) {
  if(instance_id >= 0) api()->deletePluginInstance(instance_id);
}

} // namespace mu::vst_host
