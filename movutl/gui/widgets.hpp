#pragma once
#include <movutl/core/vector.hpp>

namespace mu {
class Entity;

void wd_entt_props_editor(Entity* e);
void wd_movie_inspector(Entity* e);
void wd_image_inspector(Entity* e);
bool wd_color_edit(const char* name, Vec4b* col);

} // namespace mu
