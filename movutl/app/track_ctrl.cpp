#include <movutl/app/app.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto e = Movie::Create(name, path);
  if(!e) {
    LOG_F(ERROR, "Failed to load file: %s", path);
    return nullptr;
  }
  e->trk.fstart = start;
  e->trk.fend = start + 1; // 動画が読み込めなかった時用
  e->load_file(path);
  auto pj = Project::Get();
  Composition* main_comp = pj->get_main_comp();
  if(!main_comp) {
    Project::New();
    main_comp = pj->get_main_comp();
  }
  MU_ASSERT(main_comp);
  MU_ASSERT(layer >= 0 && layer <= 1000);
  if(layer > main_comp->layers.size()) main_comp->layers.resize(layer);
  main_comp->layers[layer].entts.push_back(e);
  return e;
}

bool add_new_audio_track(const char* name, const char* path, int start, int layer) {
  MU_FAIL("Not implemented yet");
  MU_UNUSED(name);
  MU_UNUSED(path);
  MU_UNUSED(start);
  MU_UNUSED(layer);
  return false;
}

bool add_new_track(const char* name, EntityType type, int start, int end){
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  MU_FAIL("Not implemented yet");
  /* auto pj = Project::Get(); */
  /* Composition* main_comp = pj->get_main_comp(); */
  /* if(!main_comp) { */
  /*   Project::New(); */
  /*   main_comp = pj->get_main_comp(); */
  /* } */
  /* MU_ASSERT(main_comp); */
  /* TrackLayer trk; */
  /* trk.name = name; */
  /* trk.type = type; */
  /* trk.fstart = start; */
  /* trk.fend = end; */
  /* main_comp->tracks.push_back(trk); */
  return true;
}

} // namespace mu
