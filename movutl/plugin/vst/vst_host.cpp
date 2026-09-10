#include <atomic>
#include <condition_variable>
#include <cpplocate/cpplocate.h>
#include <filesystem>
#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/vst/vst_host.hpp>
#include <mutex>
#include <remidy/remidy.hpp>
#include <thread>
#include <uapmd-plugin-hosting/uapmd-plugin-hosting.hpp>

namespace mu::vst_host {
// AudioPluginHostingAPIはカスタム検索ディレクトリを追加するAPIを公開しないため、独立のPluginFormatVST3でスキャンしplugin-list-cache.jsonへ注入する方式を採る。

namespace {
uapmd_plugin_hosting::AudioPluginHostingAPI* api() {
  static std::unique_ptr<uapmd_plugin_hosting::AudioPluginHostingAPI> instance = uapmd_plugin_hosting::AudioPluginHostingAPI::create();
  return instance.get();
}
std::atomic<bool> scanning_{false};

// uapmd-plugin-hosting/src/scanner/PluginScanTool.cppのTOOLING_DIR_NAMEと同じ文字列
constexpr const char* kToolingDirName = "remidy-tooling";

// Config::vst_plugin_dirsをスキャンし、見つかったプラグインをキャッシュファイルへマージする。
void scan_custom_dirs() {
  std::vector<std::string> override_paths;
  for(auto& dir : mu::Config::Get()->vst_plugin_dirs) {
    if(std::filesystem::exists(dir)) override_paths.push_back(dir);
  }
  if(override_paths.empty()) return;

  // movutlはUIスレッドループ未起動のためこの呼び出しスレッドをメインスレッド扱いにし、ロード待機の無限ブロックを防ぐ。
  remidy::EventLoop::initializeOnUIThread();

  auto format    = remidy::PluginFormatVST3::create(override_paths);
  auto* scanning = dynamic_cast<remidy::FileOrUrlBasedPluginScanning*>(format->scanning());
  if(!scanning) return;
  // PluginFormatVST3Implのコンストラクタ引数はscanning_へ転送されない(remidy側のバグ)ためaddSearchPath()で登録する。
  for(auto& dir : override_paths) scanning->addSearchPath(dir);

  std::vector<remidy::PluginCatalogEntry> found;
  bool done = false;
  scanning->startSlowPluginScan([&](remidy::PluginCatalogEntry entry) { found.push_back(std::move(entry)); },
                                [&](std::string error) {
                                  if(!error.empty()) LOG_F(WARNING, "vst_host: custom dir scan finished with error: %s", error.c_str());
                                  done = true;
                                });
  if(!done) return; // 同期的に返る実装のみ対応(非同期実装が入った場合は要拡張)
  if(found.empty()) return;

  auto cache_dir = cpplocate::localDir(kToolingDirName);
  if(cache_dir.empty()) return;
  std::filesystem::path cache_file = std::filesystem::path{cache_dir}.append("plugin-list-cache.json");

  remidy::PluginCatalog cat;
  cat.load(cache_file);
  for(auto& entry : found) {
    if(!cat.contains(entry.format(), entry.pluginId())) cat.add(entry);
  }
  cat.save(cache_file);
}
} // namespace

void scan_and_load(std::function<void()> on_complete) {
  if(scanning_.exchange(true)) return;
  std::thread([on_complete = std::move(on_complete)] {
    scan_custom_dirs();
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
