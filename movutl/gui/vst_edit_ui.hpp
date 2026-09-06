#pragma once

namespace uapmd_plugin_hosting {
class AudioPluginInstanceAPI;
} // namespace uapmd_plugin_hosting

namespace mu {

// VSTインスタンスの「Edit」ボタンを描画する。GUI対応プラグインはネイティブウィンドウを表示し、非対応ならImGuiスライダーでパラメータ一覧を表示する
void draw_vst_edit_button(const char* label_id, uapmd_plugin_hosting::AudioPluginInstanceAPI* inst);

} // namespace mu
