#pragma once
#include <movutl/core/platform.hpp>
#include <movutl/plugin/exdata.hpp>
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
  void (*plugin_init)(ExeData* exdata);
  void (*plugin_exit)(ExeData* exdata);
};

namespace detail {

using PluginEntryPointType = void (*)(ExeData* exdata, PluginTable* table);
struct PluginData {
  PluginTable table;
  AddonLibraryModuleT mod;
  PluginEntryPointType entry;
};

void init_plugins();

} // namespace detail

bool register_plugin(const std::string& filepath);

} // namespace mu
