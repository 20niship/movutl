#pragma once
/**
 * 色調補正などの色を調整するフィルタクラス
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_color_correction;
extern FilterPluginTable f_single_color;
extern FilterPluginTable f_color_shift;
extern FilterPluginTable f_gradient;
extern FilterPluginTable f_extend_color;
extern FilterPluginTable f_blur;
extern FilterPluginTable f_directional_blur;
extern FilterPluginTable f_radial_blur;

} // namespace mu
