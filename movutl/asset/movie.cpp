#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>

namespace mu {
static Ref<Movie> Create(const char* name, const char* path = nullptr) {
  auto mov = std::make_shared<Movie>();
  strncpy(mov->name, name, MAX_DISPNAME);
  Project::Get()->entities.push_back(mov);
  mov->guid_ = Project::Get()->entities.size();
  return mov;
}
} // namespace mu
