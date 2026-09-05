#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/filter.hpp>
#include <movutl/plugin/plugin.hpp>
//
#include <movutl/asset/audio.hpp>
#include <movutl/asset/camera.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/framebuffer.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>

namespace mu {
struct InputPluginTable;

Ref<Entity> Entity::CreateEntity(const char* name, EntityType type) {
  Ref<Entity> e = nullptr;
  switch(type) {
    case EntityType_Movie: e = cutil::make_ref<Movie>(); break;
    case EntityType_Image: e = cutil::make_ref<Image>(); break;
    case EntityType_3DText: e = cutil::make_ref<TextEntt>(); break;
    case EntityType_Audio: e = cutil::make_ref<AudioEntt>(); break;
    case EntityType_Framebuffer: e = cutil::make_ref<FramebufferEntt>(); break;
    case EntityType_Polygon: e = cutil::make_ref<ShapeEntt>(); break;
    case EntityType_Camera: e = cutil::make_ref<Camera3D>(); break;
    default: break;
  }
  if(!e) {
    LOG_F(ERROR, "[Entity::Create] Unknown type %d", type);
    return nullptr;
  }
  e->name = name;
  Project::Get()->entities.push_back(e);
  e->guid_ = Project::Get()->entities.size();
  return e;
}

Ref<Entity> Entity::Find(const char* name) {
  for(auto& e : Project::Get()->entities) {
    if(e->name == name) return e;
  }
  return nullptr;
}

cutil::Prop Entity::getSaveProps() const {
  cutil::Prop p;
  p.set<int32_t>("type", (int32_t)getType());
  p.set<std::string>("name", name.c_str());
  p.set<int32_t>("guid", (int32_t)guid_);
  p.set_child("props", getProps());
  p.set_child("trk", trk.getProps());

  // trk.filtersはTrackObject::getPropsInfo()の自動生成対象外(std::vector<FilterParam>)のため個別にシリアライズする
  cutil::Prop filters_p;
  filters_p.set<int32_t>("count", (int32_t)trk.filters.size());
  for(size_t i = 0; i < trk.filters.size(); i++) {
    const auto& f = trk.filters[i];
    cutil::Prop fp;
    fp.set<int32_t>("plugin_guid", (int32_t)(f.plg_ ? f.plg_->guid : 0));
    fp.set<bool>("enabled", f.enabled);
    fp.set_child("params", f.props.get(0));
    filters_p.set_child(("filter_" + std::to_string(i)).c_str(), fp);
  }
  p.set_child("filters", filters_p);
  return p;
}

Ref<Entity> Entity::fromSaveProps(const cutil::Prop& p) {
  auto type = (EntityType)cutil::get_or<int32_t>(p, "type", 0);
  auto name = cutil::get_or<std::string>(p, "name", "");
  auto e    = Entity::CreateEntity(name.c_str(), type);
  if(!e) return nullptr;
  e->guid_ = (uint64_t)cutil::get_or<int32_t>(p, "guid", (int32_t)e->guid_);
  if(p.contains("props")) e->setProps(p.get_child("props"));
  if(p.contains("trk")) e->trk.setProps(p.get_child("trk"));

  if(p.contains("filters")) {
    const auto& filters_p = p.get_child("filters");
    int32_t count         = cutil::get_or<int32_t>(filters_p, "count", 0);
    auto* main            = detail::AppMain::Get();
    for(int32_t i = 0; i < count; i++) {
      const auto& fp = filters_p.get_child(("filter_" + std::to_string(i)).c_str());
      int32_t guid   = cutil::get_or<int32_t>(fp, "plugin_guid", 0);
      TrackObject::FilterParam f;
      for(auto& plg : main->filters) {
        if((int32_t)plg.guid == guid) {
          f.plg_ = &plg;
          break;
        }
      }
      if(!f.plg_) {
        LOG_F(WARNING, "Entity::fromSaveProps: filter plugin (guid=%d) not found, skipping", guid);
        continue;
      }
      f.enabled = cutil::get_or<bool>(fp, "enabled", true);
      if(fp.contains("params")) f.props.add_props(fp.get_child("params"));
      e->trk.filters.push_back(f);
    }
  }

  e->reload_asset(); // pathはsetProps()でコピーされるだけなので、ここで独立した読み込みプラグインのインスタンスを持たせる
  return e;
}

Composition* Entity::get_comp() const {
  auto pj = Project::Get();
  for(int i = 0; i < pj->compos_.size(); i++) {
    for(auto& layer : pj->compos_[i]->layers) {
      for(auto& e : layer.entts) {
        if(e.get() == this) return pj->compos_[i].get();
      }
    }
  }
  return nullptr;
}

Entity::~Entity() {
  if(in_plg_ && in_handle_ && in_plg_->fn_close) in_plg_->fn_close(in_handle_);
}

std::string EntityInfo::str() const {
  char buf[256];
  sprintf(buf, "EntityInfo: Flag%d %dx%d %d frames %.3f fps", (int)flag, width, height, nframes, framerate);
  return std::string(buf);
}

bool Entity::render_filters(Composition* cmp, Image* img, int frame) {
  MU_ASSERT(cmp != nullptr);
  MOVUTL_ZONE_SCOPED_N("Entity::render_filters");
  for(int i = 0; i < trk.filters.size(); i++) {
    auto& f = trk.filters[i];
    if(!f.enabled) continue;
    MU_ASSERT(f.plg_ != nullptr);
    if(!f.plg_->fn_proc) {
      LOG_F(ERROR, "Plugin %s has no render function", f.plg_->name.c_str());
      continue;
    }
    MOVUTL_ZONE_SCOPED;
    MOVUTL_ZONE_NAME(f.plg_->name.c_str(), f.plg_->name.size());
    void* fp = f.plg_;
    FilterInData in;
    in.img            = img;
    in.compo          = cmp;
    in.entt           = this;
    in.reserve[0]     = frame; // AviUtl互換ブリッジ(obj.frame等)がフレーム番号を参照するために使う
    cutil::Prop props = f.props.get(frame);
    if(!f.plg_->fn_proc(fp, &in, f.props.get(frame))) {
      LOG_F(ERROR, "Plugin %s render failed", f.plg_->name.c_str());
      return false;
    }
  }
  return true;
}

} // namespace mu
