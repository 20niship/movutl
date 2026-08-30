#include <algorithm>
#include <movutl/app/app.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/input.hpp>

namespace mu {

Movie::Movie(const char* path) {
  if(path != nullptr && path[0] != '\0') load_file(path);
}

Ref<Movie> Movie::Create(const char* name, const char* path) {
  auto mov  = cutil::make_ref<Movie>();
  mov->name = name;
  Project::Get()->entities.push_back(mov);
  mov->guid_ = Project::Get()->entities.size();
  if(path != nullptr && path[0] != '\0') mov->load_file(path);
  return mov;
}

bool Movie::render(Composition* cmp, Image* target, int frame) {
  MOVUTL_ZONE_SCOPED_N("Movie::render");
  MU_ASSERT(cmp);
  MU_ASSERT(target);
  /// ロード失敗した動画は警告スパムを避けるため黙ってスキップ
  if(load_failed_ || in_plg_ == nullptr || in_handle_ == nullptr) return false;
  if(info.width <= 0 || info.height <= 0 || info.nframes <= 0) return false;

  int elapsed = frame - trk.fstart; // トラック上での経過フレーム数
  if(elapsed < 0 || elapsed >= trk.fend - trk.fstart) {
    if(!loop_) return false;
    elapsed %= (trk.fend - trk.fstart); // ループ再生
  }
  /// 開始フレーム(start_frame_)を起点に再生速度(speed, 100=等速)を適用した素材内フレーム位置
  int tlocal = start_frame_ + (int)(elapsed * (speed / 100.0f));
  tlocal     = std::clamp(tlocal, 0, (int)info.nframes - 1);

  if(!img_) img_ = cutil::make_ref<Image>();
  img_->resize(info.width, info.height);

  /// フレーム指定でプラグインから直接読み込む (aviutl2 方針)
  MU_ASSERT(in_plg_->fn_read_video);
  if(in_plg_->fn_read_video(in_handle_, tlocal, img_->data()) <= 0) return false;

  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  render_filters(cmp, img_.get(), frame);

  MU_ASSERT(img_);
  int base_x  = trk.anchor[0] + (cw - img_->width) / 2 + pos[0];
  int base_y  = trk.anchor[1] + (ch - img_->height) / 2 + pos[1];
  Vec2 center = Vec2(base_x, base_y) + trk.anchor;
  img_->copyto(target, center, this->scale.avg() / 100, this->rotation);

  return true;
}

bool Movie::load_file(const char* path) {
  /// 再オープン時は既存ハンドルを閉じて状態をリセット
  if(in_plg_ != nullptr && in_handle_ != nullptr) in_plg_->fn_close(in_handle_);
  in_plg_      = nullptr;
  in_handle_   = nullptr;
  info         = EntityInfo();
  load_failed_ = true;

  auto p = get_compatible_plugin(path, EntityType_Movie);
  if(p == nullptr) {
    LOG_F(ERROR, "No compatible plugin found for file: %s", path);
    return false;
  }
  this->path_ = path;

  in_handle_ = p->fn_open(path);
  if(in_handle_ == nullptr) {
    LOG_F(ERROR, "Failed to open movie file: %s", path);
    return false;
  }
  in_plg_ = p;

  if(p->fn_info_get == nullptr) {
    LOG_F(ERROR, "Plugin has no info_get function: %s", p->name);
    return false;
  }
  if(!p->fn_info_get(in_handle_, &info)) {
    LOG_F(ERROR, "Failed to get movie info: %s", path);
    return false;
  }
  if(info.width <= 0 || info.height <= 0 || info.nframes <= 0) {
    LOG_F(ERROR, "Invalid movie info: %s -> %s, plugin=%s", path, info.str().c_str(), p->name);
    return false;
  }

  this->trk.fend = info.nframes;
  load_failed_   = false;
  LOG_F(INFO, "Movie loaded: %s (%dx%d, %d frames, %.3f fps, plugin=%s)", path, info.width, info.height, info.nframes, info.framerate, p->name);
  return true;
}

} // namespace mu
