#include <movutl/app/app.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/plugin/filter.hpp>
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

Composition* Entity::get_comp() const {
  auto pj = Project::Get();
  for(int i = 0; i < pj->compos_.size(); i++) {
    for(auto& layer : pj->compos_[i].layers) {
      for(auto& e : layer.entts) {
        if(e.get() == this) return &pj->compos_[i];
      }
    }
  }
}

Entity::~Entity() {
  if(in_plg_ && in_handle_) in_plg_->fn_close(in_handle_);
}

std::string EntityInfo::str() const {
  char buf[256];
  sprintf(buf, "EntityInfo: Flag%d %dx%d %d frames %f fps", (int)flag, width, height, nframes, framerate);
  return std::string(buf);
}

bool Entity::render_filters(Composition* cmp, ImageRGBA* img) {
  MU_ASSERT(cmp != nullptr);
  for(int i = 0; i < trk.filters.size(); i++) {
    auto& f = trk.filters[i];
    if(!f.enabled) continue;
    MU_ASSERT(f.plg_ != nullptr);
    if(!f.plg_->fn_proc) {
      LOG_F(ERROR, "Plugin %s has no render function", f.plg_->name.c_str());
      continue;
    }
    void* fp = f.plg_;
    FilterInData in;
    in.img = img;
    in.compo = cmp;
    Props props = f.props.get(cmp->frame);
    if(!f.plg_->fn_proc(fp, &in, f.props.get(cmp->frame))) {
      LOG_F(ERROR, "Plugin %s render failed", f.plg_->name.c_str());
      return false;
    }
  }
  return true;
}

} // namespace mu
