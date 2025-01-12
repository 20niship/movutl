#pragma once
#include <movutl/asset/entity.hpp>
#include <vector>

namespace mu {

void init();
void update();
void terminate();
bool should_terminate();

// ------------ Plugins ------------
InputPluginTable* get_compatible_plugin(const char* path, EntityType type);

// ------------ API ------------
void new_project();
void save_project();
void save_project_as(const char* path);
void open_project(const char* path);

bool add_new_track(const char* name, EntityType type, int start, int end);

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer);
bool add_new_audio_track(const char* name, const char* path, int start, int layer);

std::vector<Ref<Entity>> get_selected_entts();
void clear_selected_entts();
void select_entt(const Ref<Entity>& entt);
void select_entts(const std::vector<Ref<Entity>>& entts);

} // namespace mu
