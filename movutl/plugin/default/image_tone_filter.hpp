#pragma once
/**
 * 単純なトーン変換フィルタ(反転/モノクロ/セピア)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_invert;
extern FilterPluginTable f_grayscale;
extern FilterPluginTable f_sepia;

} // namespace mu::detail
