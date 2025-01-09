#include <filesystem>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/plugin.hpp>

#ifdef MOVUTL_PLATFORM_WINDOWS
#include <windows.h>
#elif defined(MOVUTL_PLATFORM_MACOS)
#include <dlfcn.h>
#else
#error "Unsupported platform"
#endif

namespace mu {
namespace detail {

namespace fs = std::filesystem;

void init_plugins() {
  // xxx.mso ファイルを読み込む
  auto search_paths = Config::Get()->plugin_search_paths;
  for(const auto& path : search_paths) {
    for(const auto& entry : fs::directory_iterator(path)) {
      if(!entry.is_regular_file() || !entry.path().string().ends_with(".mso")) continue;
      register_plugin(entry.path().string());
    }
  }
}

bool register_plugin(const std::string& path) {
  // プラグインを読み込む
  AddonLibraryModuleT mod = nullptr;
#ifdef MOVUTL_PLATFORM_WINDOWS
  mod = LoadLibraryA(path.c_str());
#else
  mod = dlopen(path.c_str(), RTLD_LAZY);
#endif

  if(mod == nullptr) {
    // プラグインの読み込みに失敗
    LOG_F(ERROR, "Failed to load plugin: %s", path.c_str());
    return false;
  }

  // プラグインのエントリーポイントを取得
  PluginEntryPointType entry = nullptr;
#ifdef MOVUTL_PLATFORM_WINDOWS
  entry = (PluginEntryPointType)GetProcAddress(mod, "plugin_entry");
#else
  entry = (PluginEntryPointType)dlsym(mod, "plugin_entry");
#endif

  if(entry == nullptr) {
    LOG_F(ERROR, "Failed to find plugin entry point: %s", path.c_str());
    return false;
  }

  // プラグインのエントリーポイントを実行
  ExeData exdata;
  PluginData data;
  // TODO: get exe data
  entry(&exdata, &data.table);
  data.mod = mod;
  data.entry = entry;
  AppMain::Get()->plugins.push_back(data);
  return true;
}

} // namespace detail

} // namespace mu
