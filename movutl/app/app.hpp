#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
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
Ref<TextEntt> add_new_text_track(const char* name, int start, int end);
Ref<Image> add_new_image_track(const char* name, const char* path, int start, int end);

// 登録済みフィルタ(AppMain::filters)を名前で検索しentt->trk.filtersへ追加する(見つからなければfalse)
bool add_filter_to_entity(const Ref<Entity>& entt, const char* filter_name);
// LuaIntf上はShapeEntt等の派生クラスからEntityへ暗黙変換できないため、Luaから使うための薄いラッパー
bool add_filter_to_shape(const Ref<ShapeEntt>& entt, const char* filter_name);
bool add_filter_to_image(const Ref<Image>& entt, const char* filter_name);
// entt->trk.filters内でfilter_nameに一致する最後のフィルタのfloatパラメータをparam_nameで検索し書き換える
bool set_shape_filter_param(const Ref<ShapeEntt>& entt, const char* filter_name, const char* param_name, float value);
bool set_image_filter_param(const Ref<Image>& entt, const char* filter_name, const char* param_name, float value);
// アクティブCompositionの現在フレームをレンダリングしPNGへ書き出す(headlessスクリプト用)
bool export_current_frame_png(const char* path);
// GLFWウィンドウの現在の画面(ImGui UI込み)をキャプチャしPNGへ書き出す(GUIの動作確認用)
bool export_screen_png(const char* path);

std::vector<Ref<Entity>> get_selected_entts();
void clear_selected_entts();
void select_entt(const Ref<Entity>& entt);
void select_entts(const std::vector<Ref<Entity>>& entts);

} // namespace mu
