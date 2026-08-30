#pragma once
#include <cutil/prop.hpp>
#include <movutl/core/prop_types.hpp>

namespace mu {

// bool/int32/float/std::stringのみ扱う汎用PropInfo/Propエディタ(Entity/mtxに依存しないためgui/widgets.hppのwd_entt_props_editorとは別実装)
bool draw_props_editor(const cutil::PropInfo* info, cutil::Prop* props);

} // namespace mu
