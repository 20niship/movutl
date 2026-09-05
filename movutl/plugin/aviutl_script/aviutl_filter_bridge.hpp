#pragma once

#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>

namespace mu::detail {

// AviUtlスクリプトブロックをFilterPluginTableとしてAppMain::filtersへ登録する
bool register_aviutl_filter(AviUtlScriptDef def);

} // namespace mu::detail
