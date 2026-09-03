#pragma once
/**
 * 特定色/輝度を透明にするキー抜きフィルタ(クロマキー/ルミナンスキー)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_chroma_key;
extern FilterPluginTable f_luminance_key;

} // namespace mu::detail
