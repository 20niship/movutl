#pragma once
#include <movutl/asset/entity.hpp>

namespace mu {

void init();
void update();
void terminate();
bool should_terminate();

// ------------ API ------------
void new_project();
void save_project();
void save_project_as(const char* path);
void open_project(const char* path);

bool add_new_track(const char* name, EntityType type, int start, int end);

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer);
bool add_new_audio_track(const char* name, const char* path, int start, int layer);

} // namespace mu
