#pragma once
/**
 * その他のAviUtl標準フィルタ(ノイズ除去/シャープ/エッジ抽出/モザイク/リサイズ)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_denoise;
extern FilterPluginTable f_sharpen;
extern FilterPluginTable f_edge_detect;
extern FilterPluginTable f_mosaic;
extern FilterPluginTable f_resize;

} // namespace mu::detail
