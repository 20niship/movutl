#pragma once
#include <cstdint>

namespace mu {

struct InputPluginTable;
struct FilterPluginTable;

// PluginTable::plugin_init(Abi*) に渡され、plugin側はregister_*でAppMainへ自己登録する。
struct Abi {
  uint32_t abi_version = 1;

  bool (*register_input_plugin)(const InputPluginTable* table)   = nullptr;
  bool (*register_filter_plugin)(const FilterPluginTable* table) = nullptr;
};

} // namespace mu
