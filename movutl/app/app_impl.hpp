#pragma once
#include <imgui.h>
#include <map>
#include <movutl/core/defines.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

class AppMain {
public:
  MOVUTL_DECLARE_SINGLETON(AppMain);
  AppMain()  = default;
  ~AppMain() = default;

  std::vector<FilterPluginTable> filters;
  std::vector<InputPluginTable> input_plugins;
  std::vector<PluginData> plugins;
  std::vector<Ref<Entity>> entt_selected;

  std::map<std::string, ImGuiStyle> imgui_styles;
  std::map<std::string, Workspace> workspaces;
};

void register_default_commands();

} // namespace mu::detail
