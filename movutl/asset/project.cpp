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

Composition* Project::GetActiveCompo() {
  auto pj = Project::Get();
  if(pj->main_comp_idx < 0 || pj->main_comp_idx >= pj->compos_.size()) return nullptr;
  return &pj->compos_[pj->main_comp_idx];
}

void Project::SetActiveCompo(int idx) {
  auto pj = Project::Get();
  pj->main_comp_idx = idx;
}

Project* Project::singleton_ = nullptr;

} // namespace mu
