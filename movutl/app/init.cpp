#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/binding/binding.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/plugin.hpp>

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
  detail::register_custom_objects();
  LOG_F(1, "Loading plugins...");
  detail::activate_all_plugins();
  Config::Load();
  LOG_F(1, "Initialization complete");
  detail::init_lua_binding();
}

} // namespace mu
