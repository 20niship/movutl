#pragma once
#include <cstdint>

namespace mu {

struct InputPluginTable;
struct FilterPluginTable;
struct OutputPluginTable;

// PluginTable::plugin_init(ABIContext*)に渡す。フィールドはpygen(AbiWriter)がplugin.hppの"// MABI_FUNC"マーカーから自動生成。
struct ABIContext {
  uint32_t abi_version = 1;
#include <movutl/generated/generated_abi_fields.inc>
};

} // namespace mu
