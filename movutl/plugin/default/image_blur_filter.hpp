#pragma once
/**
 * ぼかし系フィルタ(ぼかし/方向ぼかし/放射ぼかし)
 */
#include <movutl/plugin/filter.hpp>

namespace mu::detail {

extern FilterPluginTable f_blur;
extern FilterPluginTable f_directional_blur;
extern FilterPluginTable f_radial_blur;

} // namespace mu::detail
