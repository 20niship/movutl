#pragma once

#include <movutl/asset/entity.hpp>
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
  std::vector<Ref<Entity>> entities;
};

} // namespace mu
