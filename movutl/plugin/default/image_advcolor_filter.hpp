#pragma once
/**
 * 高度な色補正系フィルタ(カラーバランス/カラーLUT/ソフトフォーカス/インターレースシフト)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_color_balance;
extern FilterPluginTable f_color_lut;
extern FilterPluginTable f_soft_focus;
extern FilterPluginTable f_interlace_shift;

} // namespace mu::detail
