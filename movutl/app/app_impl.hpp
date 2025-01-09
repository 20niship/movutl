#pragma once
#include <movutl/core/defines.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

class AppMain {
public:
  MOVUTL_DECLARE_SINGLETON(AppMain);
  AppMain();
  ~AppMain();

  void init();
  void exit();

  std::unordered_map<uint64_t, FilterPluginTable*> filters;
  std::unordered_map<uint64_t, InputPluginTable*> input_plugins;
  std::vector<PluginData> plugins;
};

} // namespace mu::detail
