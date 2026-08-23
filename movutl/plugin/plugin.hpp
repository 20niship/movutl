#pragma once
#include <movutl/core/platform.hpp>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/filter.hpp>
#include <movutl/plugin/input.hpp>

#ifdef MOVUTL_PLATFORM_WINDOWS
#include <windows.h>
using AddonLibraryModuleT = HMODULE;
#else
#include <dlfcn.h>
using AddonLibraryModuleT = void*;
#endif

namespace mu {

struct PluginTable {
  char name[64];
  char description[256];
  char filter[256];
  void (*plugin_init)(ABIContext* abi);
  void (*plugin_exit)(ABIContext* abi);
};

namespace detail {

using PluginEntryPointType = void (*)(ABIContext* abi, PluginTable* table);
struct PluginData {
  PluginTable table;
  AddonLibraryModuleT mod;
  PluginEntryPointType entry;
};

void init_external_plugins();
void activate_all_plugins();

// pygen(AbiWriter)がここを解析しABIContextのフィールドとmake_abi()を自動生成する。増やす場合はここに1行足すだけでよい。
bool abi_register_input_plugin(const InputPluginTable* table);   // MABI_FUNC
bool abi_register_filter_plugin(const FilterPluginTable* table); // MABI_FUNC
ABIContext make_abi();                                           // 実装は movutl/generated/generated_abi.cpp (pygen生成)

} // namespace detail

bool register_plugin(const std::string& filepath);

namespace detail {
void register_default_plugins();
void register_default_filters();
} // namespace detail
} // namespace mu
