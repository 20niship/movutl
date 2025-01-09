#include <movutl/app/app_impl.hpp>
#include <movutl/app/ui.hpp>
#include <movutl/app/app.hpp>
#include <movutl/asset/config.hpp>

namespace mu{


void init(){
  printf("movutl project !!\n");
  GUIManager::Get()->init();
  printf("GUIManager::Get()->init();\n");
  Config::Load();
  printf("Config::Load();\n");
}

}
