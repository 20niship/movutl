#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

// uapmd-plugin-hosting(AudioPluginHostingAPI)の薄いラッパ。VST3以外(LV2/CLAP)はビルドに含まれるが対象外とする。
namespace uapmd_plugin_hosting {
class AudioPluginHostingAPI;
class AudioPluginInstanceAPI;
} // namespace uapmd_plugin_hosting

namespace mu::vst_host {

struct PluginInfo {
  std::string id;     // AudioPluginHostingAPI::createPluginInstanceへ渡すpluginId
  std::string format; // 常に"VST3"
  std::string name;   // 表示名
};

// バックグラウンドスレッドでVST3をスキャンする(起動をブロックしない、二重起動は無視)。on_completeはそのスレッド上で呼ばれる。
void scan_and_load(std::function<void()> on_complete = nullptr);
bool is_scanning();

// スキャン済みのVST3プラグイン一覧(未スキャン/スキャン中は空になりうる)
std::vector<PluginInfo> plugin_list();

// プラグインインスタンスを生成する。失敗時は-1を返す(この呼び出しはコールバック完了までブロックする)。
int32_t create_instance(const std::string& plugin_id, uint32_t sample_rate, uint32_t buffer_size);
uapmd_plugin_hosting::AudioPluginInstanceAPI* get_instance(int32_t instance_id);
void destroy_instance(int32_t instance_id);

} // namespace mu::vst_host
