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

Ref<Entity> Entity::Create(const char* name, EntityType type) {
  auto e = std::make_shared<Entity>();
  switch(type) {
    case EntityType_Movie: e = std::make_shared<Movie>(); break;
    case EntityType_Image: e = std::make_shared<Image>(); break;
    default: break;
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

Ref<Entity> Entity::LoadFile(const char* name, const char* path) {
  auto e = Create(name, EntityType_Image);
  if(!e) return nullptr;
  return e;
}

} // namespace mu
