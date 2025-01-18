#include <movutl/app/app.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>

namespace mu {
Ref<Movie> Movie::Create(const char* name, const char* path) {
  auto mov = std::make_shared<Movie>();
  mov->name = name;
  Project::Get()->entities.push_back(mov);
  mov->guid_ = Project::Get()->entities.size();
  // TODO: Load movie from file
  return mov;
}

bool Movie::render(Composition* cmp) {
  if(!in_plg_ || info.width <= 0 || info.height <= 0 || info.nframes <= 0) {
    LOG_F(WARNING, "Invalid movie info: %s -> %s", name.c_str(), info.str().c_str());
    return false;
  }

  MU_ASSERT(cmp);
  MU_ASSERT(cmp->frame_final);
  int tlocal = cmp->frame - trk.fstart;
  if(tlocal < 0 || tlocal >= trk.fend - trk.fstart) return false;
  if(in_plg_->fn_set_frame(in_handle_, tlocal)) in_plg_->fn_set_frame(in_handle_, tlocal);

  if(!img_) img_ = Image::Create("!movie", info.width, info.height, ImageFormatRGBA, false);
  MU_ASSERT(in_plg_->fn_read_video);
  in_plg_->fn_read_video(in_handle_, &info, this);
  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  MU_ASSERT(img_);
  MU_ASSERT(img_->fmt == ImageFormatRGBA);
  int base_x = trk.anchor[0] + (cw - img_->width()) / 2 + pos[0];
  int base_y = trk.anchor[1] + (ch - img_->height()) / 2 + pos[1];
  Vec2 center = Vec2(base_x, base_y) + trk.anchor;
  img_->copyto(cmp->frame_final.get(), center, this->scale.avg() / 100, this->rotation);

  cmp->frame_final->dirty();
  return true;
}

bool Movie::load_file(const char* path) {
  auto p = get_compatible_plugin(path, EntityType_Movie);
  if(!p) {
    LOG_F(WARNING, "No compatible plugin found for file: %s", path);
    return false;
  }
  this->in_plg_ = p;
  this->in_handle_ = p->fn_open(path);
  if(!p->fn_info_get)
    LOG_F(WARNING, "No info_get function found for plugin: %s", p->name);
  else
    p->fn_info_get(in_handle_, &info);
  if(info.width <= 0 || info.height <= 0 || info.nframes <= 0) {
    LOG_F(WARNING, "Invalid movie info: %s -> %s, plugin=%s", path, info.str().c_str(), p->name);
    return false;
  }
  this->trk.fend = info.nframes;
  return true;
}

} // namespace mu
