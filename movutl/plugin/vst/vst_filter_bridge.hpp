#pragma once
#include <cstdint>

namespace uapmd_plugin_hosting {
class AudioPluginInstanceAPI;
} // namespace uapmd_plugin_hosting

namespace mu::detail {

// register_vst_filters()がFilterParam::instance_stateに割り当てる状態(インスペクタのEditボタンからも参照するため公開)
struct VstFilterState {
  int32_t instance_id = -1;
};

// f.instance_state(未生成ならnullptr)から実際に対応するVSTインスタンスを取得する。取得できなければnullptr。
uapmd_plugin_hosting::AudioPluginInstanceAPI* vst_filter_instance(void* instance_state);

// このFilterPluginTableがregister_vst_filters()が登録したVSTエフェクトかどうか
bool is_vst_filter_guid(uint64_t guid);

} // namespace mu::detail
