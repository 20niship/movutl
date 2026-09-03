#include <movutl/app/app.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

Ref<ShapeEntt> add_new_shape_track(const char* name, int start, int end, ShapeType type) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  auto shp               = ShapeEntt::Create(name, type);
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  shp->trk.fstart = start;
  shp->trk.fend   = end;
  main_comp->insert_entity(shp);
  return shp;
}

Ref<TextEntt> add_new_text_track(const char* name, int start, int end) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  auto txt               = TextEntt::Create(name);
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  txt->trk.fstart = start;
  txt->trk.fend   = end;
  main_comp->insert_entity(txt);
  return txt;
}

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto e = Movie::Create(name, path);
  if(!e) {
    LOG_F(ERROR, "Failed to load file: %s", path);
    return nullptr;
  }
  e->trk.fstart = start;
  e->trk.fend   = start + 1; // 動画が読み込めなかった時用
  e->load_file(path);
  auto pj                = Project::Get();
  Composition* main_comp = pj->get_main_comp();
  if(!main_comp) {
    Project::New();
    main_comp = pj->get_main_comp();
  }
  MU_ASSERT(main_comp);
  MU_ASSERT(layer >= 0 && layer <= 1000);
  if(layer >= (int)main_comp->layers.size()) main_comp->layers.resize(layer + 1);
  main_comp->layers[layer].entts.push_back(e);
  return e;
}

bool add_new_audio_track(const char* name, const char* path, int start, int layer) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto pj                = Project::Get();
  Composition* main_comp = pj->get_main_comp();
  if(!main_comp) {
    Project::New();
    main_comp = pj->get_main_comp();
  }
  MU_ASSERT(main_comp);
  MU_ASSERT(layer >= 0 && layer <= 1000);

  // Composition未確定のままload_file()するとGetActiveComp()がnullptrでframerateが既定値30にフォールバックしend frameがずれるため、先にtrk.fstartを設定してからload_file()する
  auto e = AudioEntt::Create(name);
  if(!e) {
    LOG_F(ERROR, "Failed to create audio entity: %s", path);
    return false;
  }
  e->trk.fstart = start;
  e->trk.fend   = start + 1; // 読み込み失敗時用
  if(!e->load_file(path)) LOG_F(ERROR, "Failed to load audio file: %s", path);

  if(layer >= (int)main_comp->layers.size()) main_comp->layers.resize(layer + 1);
  main_comp->layers[layer].entts.push_back(e);
  return true;
}

bool add_new_track(const char* name, EntityType type, int start, int end) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  switch(type) {
    case EntityType_Image: {
      auto img               = Image::Create(name, "");
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      img->trk.fstart = start;
      img->trk.fend   = end;
      main_comp->insert_entity(img);
      break;
    }
    case EntityType_Movie: {
      auto mov               = Movie::Create(name, "");
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      mov->trk.fstart = start;
      mov->trk.fend   = end;
      main_comp->insert_entity(mov);
      break;
    }
    case EntityType_3DText: {
      auto txt               = TextEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      txt->trk.fstart = start;
      txt->trk.fend   = end;
      main_comp->insert_entity(txt);
      break;
    }
    case EntityType_Polygon: {
      auto shp               = ShapeEntt::Create(name, ShapeType_Rect);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      shp->trk.fstart = start;
      shp->trk.fend   = end;
      main_comp->insert_entity(shp);
      break;
    }
    case EntityType_Audio: {
      auto a                 = AudioEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      a->trk.fstart = start;
      a->trk.fend   = end;
      main_comp->insert_entity(a);
      break;
    }
    case EntityType_Framebuffer: {
      auto fb                = FramebufferEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      fb->trk.fstart = start;
      fb->trk.fend   = end;
      main_comp->insert_entity(fb);
      break;
    }
    default: MU_FAIL("Not implemented yet"); break;
  }
  return true;
}

} // namespace mu
