#include <movutl/app/app.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer) {
  auto e = Entity::LoadFile(name, path);
  if(!e){
    LOG_F(ERROR, "Failed to load file: %s", path);
    return nullptr;
  }
  return e;
}

bool add_new_audio_track(const char* name, const char* path, int start, int layer) {
  return false;
}
} // namespace mu
