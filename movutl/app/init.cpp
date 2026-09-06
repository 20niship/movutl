#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

void init() {
  detail::enable_signal_handlers();
  detail::init_logger();
  // レンダー/音声ワーカーがdangling Composition*を握ったままProject::New/Loadがcompos_.clear()しないよう静止させる
  Project::SetWorkerQuiesceHook([] {
    auto* app = detail::AppMain::Get();
    app->render_pool.flush();
    app->audio_worker.pause();
    app->audio_player.stop(); // ma_device_stop()はコールバックスレッド停止まで待つため、再生デバイスのdangling参照も防げる
  });
  GUIManager::Get()->init();
  LOG_F(1, "Loading plugins...");
  detail::register_default_plugins();
  detail::register_default_filters();
  detail::register_default_commands();
  detail::init_external_plugins();
  detail::register_aviutl_scripts();
  detail::register_custom_objects();
  LOG_F(1, "Loading plugins...");
  detail::activate_all_plugins();
  Config::Load();
  LOG_F(1, "Initialization complete");
  detail::init_lua_binding();
}

} // namespace mu
