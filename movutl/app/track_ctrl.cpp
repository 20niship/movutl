#include <filesystem>
#include <movutl/app/app.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/custom_object.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/midi.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu {

namespace {
// EntityType_Movie対応プラグインは拡張子だけで判定するため音声専用ファイルも「対応」を返す。開いて実際に確認する
bool has_video_stream_impl(const char* path) {
  auto* plg = get_compatible_plugin(path, EntityType_Movie);
  if(!plg) return false;
  InputHandle h = plg->fn_open(path);
  if(!h) return false;
  EntityInfo info;
  bool ok = plg->fn_info_get && plg->fn_info_get(h, &info) && info.width > 0 && info.height > 0 && info.nframes > 0;
  plg->fn_close(h);
  return ok;
}
} // namespace

Ref<ShapeEntt> add_new_shape_track(const char* name, int start, int end, ShapeType type) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  auto shp               = ShapeEntt::Create(name, type);
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  shp->fstart_ = start;
  shp->fend_   = end;
  main_comp->insert_entity(shp);
  return shp;
}

Ref<Image> add_new_image_track(const char* name, const char* path, int start, int end) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto img = Image::Create(name, path);
  if(!img || img->width == 0) {
    LOG_F(ERROR, "Failed to load image: %s", path);
    return nullptr;
  }
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  img->fstart_ = start;
  img->fend_   = end;
  main_comp->insert_entity(img);
  return img;
}

Ref<TextEntt> add_new_text_track(const char* name, int start, int end) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(start >= 0);
  MU_ASSERT(end >= start);
  auto txt               = TextEntt::Create(name);
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  txt->fstart_ = start;
  txt->fend_   = end;
  main_comp->insert_entity(txt);
  return txt;
}

Ref<Entity> add_new_custom_object_track(const std::string& script_name, int start, int end) {
  auto e = CustomObjectEntt::Create(script_name.c_str(), script_name); // nameとscript_nameは同じでよい(ユーザーは後でEntity::nameを改名できる)
  if(!e) return nullptr;
  Composition* main_comp = Composition::GetActiveComp();
  MU_ASSERT(main_comp);
  e->fstart_ = start;
  e->fend_   = end;
  main_comp->insert_entity(e);
  return e;
}

Ref<Entity> add_new_video_track(const char* name, const char* path, int start, int layer) {
  MU_ASSERT(name != nullptr);
  MU_ASSERT(path != nullptr);
  auto e = Movie::Create(name, path);
  if(!e) {
    LOG_F(ERROR, "Failed to load file: %s", path);
    return nullptr;
  }
  e->fstart_ = start;
  e->fend_   = start + 1; // 動画が読み込めなかった時用
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
  e->fstart_ = start;
  e->fend_   = start + 1; // 読み込み失敗時用
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
      img->fstart_ = start;
      img->fend_   = end;
      main_comp->insert_entity(img);
      break;
    }
    case EntityType_Movie: {
      auto mov               = Movie::Create(name, "");
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      mov->fstart_ = start;
      mov->fend_   = end;
      main_comp->insert_entity(mov);
      break;
    }
    case EntityType_3DText: {
      auto txt               = TextEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      txt->fstart_ = start;
      txt->fend_   = end;
      main_comp->insert_entity(txt);
      break;
    }
    case EntityType_Polygon: {
      auto shp               = ShapeEntt::Create(name, ShapeType_Rect);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      shp->fstart_ = start;
      shp->fend_   = end;
      main_comp->insert_entity(shp);
      break;
    }
    case EntityType_Audio: {
      auto a                 = AudioEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      a->fstart_ = start;
      a->fend_   = end;
      main_comp->insert_entity(a);
      break;
    }
    case EntityType_Midi: {
      auto m                 = MidiEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      m->fstart_ = start;
      m->fend_   = end;
      main_comp->insert_entity(m);
      break;
    }
    case EntityType_Framebuffer: {
      auto fb                = FramebufferEntt::Create(name);
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      fb->fstart_ = start;
      fb->fend_   = end;
      main_comp->insert_entity(fb);
      break;
    }
    case EntityType_Scene: {
      auto e                 = cutil::make_ref<CompoRefEntt>();
      e->name                = name;
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      e->fstart_ = start;
      e->fend_   = end;
      main_comp->insert_entity(e);
      break;
    }
    case EntityType_SceneAudio: {
      auto e                 = cutil::make_ref<CompoAudioEntt>();
      e->name                = name;
      Composition* main_comp = Composition::GetActiveComp();
      MU_ASSERT(main_comp);
      e->fstart_ = start;
      e->fend_   = end;
      main_comp->insert_entity(e);
      break;
    }
    default: MU_FAIL("Not implemented yet"); break;
  }
  return true;
}

Ref<Entity> import_media_file(const char* path) {
  MU_ASSERT(path != nullptr);
  Composition* main_comp = Composition::GetActiveComp();
  if(!main_comp) {
    Project::New();
    main_comp = Composition::GetActiveComp();
  }
  MU_ASSERT(main_comp);
  int start        = main_comp->frame;
  std::string base = std::filesystem::path(path).stem().string();
  // insertable_layer_index()は空きレイヤーが無いと-1を返すため、add_new_video/audio_trackのlayer>=0制約に合わせて末尾に追加する
  int layer = main_comp->insertable_layer_index();
  if(layer < 0) layer = (int)main_comp->layers.size();
  if(has_video_stream_impl(path)) return add_new_video_track(base.c_str(), path, start, layer);
  if(get_compatible_plugin(path, EntityType_Audio)) {
    if(!add_new_audio_track(base.c_str(), path, start, layer)) return nullptr;
    return Entity::Find(base.c_str());
  }
  if(get_compatible_plugin(path, EntityType_Image)) return add_new_image_track(base.c_str(), path, start, start + Config::Get()->default_image_frames);
  LOG_F(ERROR, "import_media_file: No compatible plugin found for file: %s", path);
  return nullptr;
}

} // namespace mu
