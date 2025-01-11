#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer) {
  MU_ASSERT(layer);
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto e = Entity::LoadFile(name, path);
  if(!e) {
    LOG_F(ERROR, "Failed to load file: %s", path);
    return nullptr;
  }
  auto pj = Project::Get();
  Composition* main_comp = pj->get_main_comp();
  if(!main_comp) {
    Project::New();
    main_comp = pj->get_main_comp();
  }

  MU_ASSERT(main_comp);
  if(layer > main_comp->layers.size()) main_comp->layers.resize(layer);
  main_comp->layers[layer - 1].objs.push_back(e);
  return e;
}

bool add_new_audio_track(const char* name, const char* path, int start, int layer) {
  MU_FAIL("Not implemented yet");
  return false;
}
} // namespace mu
