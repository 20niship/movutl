#pragma once
/**
 * グラデーション/クリッピング系フィルタ
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_four_color_gradient;
extern FilterPluginTable f_radial_gradient;
extern FilterPluginTable f_diagonal_clip;
extern FilterPluginTable f_circle_clip;

} // namespace mu::detail
