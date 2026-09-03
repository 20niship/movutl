#pragma once
/**
 * 座標歪み系フィルタ(レンズ歪み/波紋/揺らぎ/万華鏡)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_lens_distortion;
extern FilterPluginTable f_ripple;
extern FilterPluginTable f_wave_distortion;
extern FilterPluginTable f_kaleidoscope;

} // namespace mu::detail
