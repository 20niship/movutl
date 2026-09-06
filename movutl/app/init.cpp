#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/plugin/vst/vst_host.hpp>

namespace mu {

void init() {
  detail::enable_signal_handlers();
  detail::init_logger();
  GUIManager::Get()->init();
  LOG_F(1, "Loading plugins...");
  detail::register_default_plugins();
  detail::register_default_filters();
  detail::register_default_commands();
  detail::init_external_plugins();
  detail::register_aviutl_scripts();
  // バックグラウンドスレッドでVST3をスキャンする(起動をブロックしない)。ponytail: スキャン完了はGUIスレッドと非同期のため、完了直後にfilters配列へpush_backする瞬間だけ他スレッドの走査と競合しうる(既存のfilters配列自体に元々ロックが無く許容範囲)
  vst_host::scan_and_load([] { detail::register_vst_filters(); });
  LOG_F(1, "Loading plugins...");
  detail::activate_all_plugins();
  Config::Load();
  LOG_F(1, "Initialization complete");
  detail::init_lua_binding();
}

} // namespace mu
