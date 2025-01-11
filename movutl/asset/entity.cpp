#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/plugin/plugin.hpp>
//
#include <movutl/asset/camera.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>

namespace mu {
struct InputPluginTable;

Ref<Entity> Entity::CreateEntity(const char* name, EntityType type) {
  Ref<Entity> e = nullptr;
  switch(type) {
    case EntityType_Movie: e = std::make_shared<Movie>(); break;
    case EntityType_Image: e = std::make_shared<Image>(); break;
    default: break;
  }
  if(!e) {
    LOG_F(ERROR, "[Entity::Create] Unknown type %d", type);
    return nullptr;
  }
  strncpy(e->name, name, MAX_DISPNAME);
  Project::Get()->entities.push_back(e);
  e->guid_ = Project::Get()->entities.size();
  return e;
}

Ref<Entity> Entity::Find(const char* name) {
  for(auto& e : Project::Get()->entities) {
    if(strncmp(e->name, name, MAX_DISPNAME) == 0) return e;
  }
  return nullptr;
}

} // namespace mu
