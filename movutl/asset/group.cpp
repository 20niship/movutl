#include <movutl/asset/group.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

Ref<GroupEntt> GroupEntt::Create(const char* name) {
  auto g  = cutil::make_ref<GroupEntt>();
  g->name = name;
  Project::Get()->entities.push_back(g);
  g->guid_ = Project::Get()->entities.size();
  return g;
}

} // namespace mu
