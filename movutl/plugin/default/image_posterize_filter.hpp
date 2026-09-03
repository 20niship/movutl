#pragma once
/**
 * 階調変換系フィルタ(ポスタリゼーション/2値化/エンボス/ハーフトーン)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_posterize;
extern FilterPluginTable f_binarize;
extern FilterPluginTable f_emboss;
extern FilterPluginTable f_halftone;

} // namespace mu::detail
