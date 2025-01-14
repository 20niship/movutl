#pragma once
#include <movutl/core/defines.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

class AppMain {
public:
  MOVUTL_DECLARE_SINGLETON(AppMain);
  AppMain() = default;
  ~AppMain() = default;

  std::vector<FilterPluginTable> filters;
  std::vector<InputPluginTable> input_plugins;
  std::vector<PluginData> plugins;
  std::vector<Ref<Entity>> entt_selected;
};

} // namespace mu::detail
