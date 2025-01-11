#pragma once

#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/track.hpp>
#include <movutl/core/defines.hpp>
#include <movutl/core/ref.hpp>
#include <movutl/core/vector.hpp>
#include <string>

namespace mu {

class Project {
public:
  Project() = default;
  ~Project() = default;
  MOVUTL_DECLARE_SINGLETON(Project);

  std::string path;
  std::string output_path; //	出力ファイル名 (ファイル名が決まっていない時は何も入っていません)

  std::vector<Ref<Entity>> entities;
  std::vector<Composition> compos_;
  int main_comp_idx = -1;

  void init();
};

} // namespace mu
