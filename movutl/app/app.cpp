#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/core/filesystem.hpp>

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

std::vector<Ref<Entity>> get_selected_entts() {
  return detail::AppMain::Get()->entt_selected;
}
void clear_selected_entts() {
  detail::AppMain::Get()->entt_selected.clear();
}
void select_entt(const Ref<Entity>& entt) {
  detail::AppMain::Get()->entt_selected.push_back(entt);
}
void select_entts(const std::vector<Ref<Entity>>& entts) {
  detail::AppMain::Get()->entt_selected = entts;
}

} // namespace mu
