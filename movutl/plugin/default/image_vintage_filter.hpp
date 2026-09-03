#pragma once
/**
 * ビネット/フィルムグレイン/走査線/VHSノイズ(フィルム/レトロ映像風)フィルタ
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_vignette;
extern FilterPluginTable f_film_grain;
extern FilterPluginTable f_scanline;
extern FilterPluginTable f_vhs_noise;

} // namespace mu::detail
