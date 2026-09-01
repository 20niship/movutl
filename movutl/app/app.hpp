#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/asset/shape.hpp>
#include <vector>

namespace mu {
namespace detail {
void update_renderer_thread();
void update_audio_thread();
} // namespace detail

void init();
void update();
void terminate();
bool should_terminate();

// 指定したLuaファイルを実行する(失敗時はfalseを返す)
bool run_lua_file(const char* path);

// ------------ Plugins ------------
InputPluginTable* get_compatible_plugin(const char* path, EntityType type);

// ------------ API ------------
void new_project();
void save_project();
void save_project_as(const char* path);
void open_project(const char* path);

// srcを複製した新規Entityを返す(全EntityType対応、未対応の型/nullptrはnullptrを返す)
Ref<Entity> duplicate_asset(const Ref<Entity>& src);

bool add_new_track(const char* name, EntityType type, int start, int end);

// ------------ 再生制御 ------------
void play();
void pause();
void reset();
void goto_frame(int frame);
bool is_playing();

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer);
bool add_new_audio_track(const char* name, const char* path, int start, int layer);
Ref<ShapeEntt> add_new_shape_track(const char* name, int start, int end, ShapeType type);

std::vector<Ref<Entity>> get_selected_entts();
void clear_selected_entts();
void select_entt(const Ref<Entity>& entt);
void select_entts(const std::vector<Ref<Entity>>& entts);

} // namespace mu
