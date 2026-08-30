#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/time.hpp>

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
