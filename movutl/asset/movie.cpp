#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>

namespace mu {
Ref<Movie> Movie::Create(const char* name, const char* path) {
  auto mov = std::make_shared<Movie>();
  strncpy(mov->name, name, MAX_DISPNAME);
  Project::Get()->entities.push_back(mov);
  mov->guid_ = Project::Get()->entities.size();
  // TODO: Load movie from file
  return mov;
}

bool Movie::render(Composition* cmp) {
  MU_ASSERT(cmp);
  MU_ASSERT(cmp->frame_final);
  if(!img_) return false;
  if(img_->width <= 0 || img_->height <= 0) return false;
  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  MU_ASSERT(img_);
  int base_x = img_->width / 2 + trk.anchor_x - cw / 2;
  int base_y = img_->height / 2 + trk.anchor_y - ch / 2;
  img_->copyto(cmp->frame_final.get(), Vec2d(base_x, base_y));
  return true;
}

} // namespace mu
