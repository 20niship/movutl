#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/time.hpp>
#include <movutl/plugin/aviutl_script/aviutl_filter_bridge.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <sstream>

namespace mu {

namespace detail {
AppMain* AppMain::singleton_ = nullptr;
} // namespace detail

InputPluginTable* get_compatible_plugin(const char* path, EntityType type) {
  auto ext = fs_extension(path);
  auto app = detail::AppMain::Get();
  for(auto& plg : app->input_plugins) {
    if(plg.ext_supports(ext.c_str()) && plg.is_supports(type)) return &plg;
  }
  return nullptr;
}

std::vector<Ref<Entity>> get_selected_entts() { return detail::AppMain::Get()->entt_selected; }
void clear_selected_entts() { detail::AppMain::Get()->entt_selected.clear(); }
void select_entt(const Ref<Entity>& entt) { detail::AppMain::Get()->entt_selected.push_back(entt); }
void select_entts(const std::vector<Ref<Entity>>& entts) { detail::AppMain::Get()->entt_selected = entts; }

namespace detail {
void update_renderer_thread() {
  MOVUTL_ZONE_SCOPED_N("update_renderer_thread");
  auto cmp = Composition::GetActiveComp();
  if(!cmp) return;
  auto* app = AppMain::Get();
  app->render_pool.tick(cmp, app->is_playing());
}

void update_audio_thread() {
  MOVUTL_ZONE_SCOPED_N("update_audio_thread");
  auto cmp = Composition::GetActiveComp();
  if(!cmp || !cmp->audio_buf) return;
  auto* app          = AppMain::Get();
  const bool playing = app->is_playing();
  app->audio_worker.tick(cmp, playing);

  if(app->audio_player.is_running() != playing) {
    app->audio_player.set_composition(cmp);
    if(playing) {
      cmp->audio_buf->seek(cmp->frame_to_sample(cmp->get_frame())); // 再生開始位置をViewerの現在フレームへ合わせる
      app->audio_player.start();
    } else {
      app->audio_player.stop();
    }
    return;
  }

  if(playing) {
    // 前方への手動シーク(0.2秒以上先へ飛んだ)のみ追従。expect<currentは単に描画が実時間に遅れているだけなので巻き戻さない(巻き戻すと毎フレーム音声が止まり音切れする)
    int64_t expect  = cmp->frame_to_sample(cmp->get_frame());
    int64_t current = cmp->audio_buf->read_cursor();
    if(expect - current > cmp->audio_sample_rate / 5) cmp->audio_buf->seek(expect);
  }
}

void AppMain::play() {
  playing_         = true;
  last_frame_time_ = mu_now_seconds();
}

void AppMain::pause() { playing_ = false; }

void AppMain::reset() {
  playing_ = false;
  auto cmp = Composition::GetActiveComp();
  if(!cmp) return;
  cmp->frame = cmp->fstart;
}

void AppMain::goto_frame(int frame) {
  auto cmp = Composition::GetActiveComp();
  if(!cmp) return;
  cmp->frame = std::clamp(frame, cmp->fstart, cmp->fend);
  // Viewer/タイムラインクリック等の明示的なシークはここを必ず通るので、ここで直接音声を追従させる(推測に頼らない)
  if(cmp->audio_buf) cmp->audio_buf->seek(cmp->frame_to_sample(cmp->get_frame()));
}

void AppMain::update_frame_impl() {
  MOVUTL_ZONE_SCOPED_N("AppMain::update_frame_impl");
  if(!playing_) return;
  auto cmp = Composition::GetActiveComp();
  if(!cmp) return;

  double now = mu_now_seconds();
  double fps = (double)cmp->framerate;
  if(now - last_frame_time_ >= 1.0 / fps) {
    cmp->frame++;
    if(cmp->frame > cmp->fend) cmp->frame = cmp->fstart;
    last_frame_time_ = now;
  }
}

} // namespace detail

void play() { detail::AppMain::Get()->play(); }
void pause() { detail::AppMain::Get()->pause(); }
void reset() { detail::AppMain::Get()->reset(); }
void goto_frame(int frame) { detail::AppMain::Get()->goto_frame(frame); }
bool is_playing() { return detail::AppMain::Get()->is_playing(); }

void new_project() { Project::New(); }
void save_project() { Project::Save(); }
void save_project_as(const char* path) { Project::Save(path); }

void open_project(const char* path) { Project::Load(path); }

namespace {
bool add_filter_impl(Entity* e, const char* filter_name) {
  if(!e || !filter_name) return false;
  for(auto& plg : detail::AppMain::Get()->filters) {
    if(std::string(plg.name.c_str()) != filter_name) continue;
    TrackObject::FilterParam fp;
    fp.plg_ = &plg;
    fp.props.add_props(plg.defaults);
    fp.enabled = true;
    e->trk.filters.push_back(fp);
    return true;
  }
  LOG_F(ERROR, "add_filter_to_entity: filter not found: %s", filter_name);
  return false;
}

bool set_filter_param_impl(Entity* e, const char* filter_name, const char* param_name, float value) {
  if(!e || !filter_name || !param_name) return false;
  for(auto& f : e->trk.filters) {
    if(!f.plg_ || std::string(f.plg_->name.c_str()) != filter_name) continue;
    for(size_t i = 0; i < f.props.size(); i++) {
      bool matched = std::visit([&](auto&& clip) { return clip.keyname == param_name; }, f.props[i]);
      if(!matched) continue;
      f.props.set_value<float>((int)i, 0, value);
      return true;
    }
  }
  return false;
}
} // namespace

bool add_filter_to_entity(const Ref<Entity>& entt, const char* filter_name) { return add_filter_impl(entt.get(), filter_name); }
bool add_filter_to_shape(const Ref<ShapeEntt>& entt, const char* filter_name) { return add_filter_impl(entt.get(), filter_name); }
bool add_filter_to_image(const Ref<Image>& entt, const char* filter_name) { return add_filter_impl(entt.get(), filter_name); }

bool set_shape_filter_param(const Ref<ShapeEntt>& entt, const char* filter_name, const char* param_name, float value) { return set_filter_param_impl(entt.get(), filter_name, param_name, value); }
bool set_image_filter_param(const Ref<Image>& entt, const char* filter_name, const char* param_name, float value) { return set_filter_param_impl(entt.get(), filter_name, param_name, value); }

bool export_current_frame_png(const char* path) {
  auto cmp = Composition::GetActiveComp();
  if(!cmp) return false;
  auto img = cmp->render_current_frame_main_thread();
  if(!img) return false;

  OutputPluginTable* plg = nullptr;
  for(auto& p : detail::AppMain::Get()->output_plugins) {
    if(std::string(p.name) == "PNG Image/Sequence") {
      plg = &p;
      break;
    }
  }
  if(!plg || !plg->fn_init(&plg->props, &plg->defaults)) return false;
  void* handle = plg->fn_open(path, img->width, img->height, cmp->framerate, 0, 0, plg->defaults);
  if(!handle) return false;
  bool ok = plg->fn_write_frame(handle, img.get(), 0);
  ok      = plg->fn_close(handle) && ok;
  return ok;
}

bool load_aviutl_effect_script(const char* path) {
  std::ifstream ifs(path);
  if(!ifs) {
    LOG_F(ERROR, "load_aviutl_effect_script: ファイルを開けません: %s", path);
    return false;
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  auto defs = detail::parse_aviutl_script(ss.str());
  if(defs.empty()) return false;
  for(auto& def : defs) detail::register_aviutl_filter(std::move(def));
  return true;
}

Ref<Entity> duplicate_asset(const Ref<Entity>& src) {
  if(!src) return nullptr;
  auto clone = Entity::fromSaveProps(src->getSaveProps());
  if(!clone) {
    LOG_F(WARNING, "duplicate_asset: unsupported entity type for '%s'", src->name.c_str());
    return nullptr;
  }
  clone->guid_ = Project::Get()->entities.size(); // 複製なのでguidは振り直す
  return clone;
}

} // namespace mu
