#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

void Project::New(int width, int height, int fps) {
  auto pj = Project::Get();
  pj->compos_.clear();
  pj->entities.clear();

  auto cmp = Composition("Main", width, height, fps);
  pj->compos_.push_back(cmp);
  pj->main_comp_idx = 0;
}

} // namespace mu
