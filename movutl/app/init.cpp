#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/app/ui.hpp>
#include <movutl/asset/config.hpp>

namespace mu {


void init() {
  printf("movutl project !!\n");
  GUIManager::Get()->init();
  printf("GUIManager::Get()->init();\n");
  LOG_F(1, "Loading plugins...");
  detail::register_default_plugins();
  Config::Load();
  printf("Config::Load();\n");
}

} // namespace mu
