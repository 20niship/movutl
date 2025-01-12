#define NOMINMAX

#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

Ref<Entity> TrackLayer::find_entt(uint32_t frame) const {
  for(auto& e : entts)
    if(e->visible(frame)) return e;
  return nullptr;
}


std::string TrackLayer::str() const {
  std::string str = "Layer<" + std::string(name) + " / entt:" + std::to_string(entts.size()) + ">";
  return str;
}
std::string TrackLayer::summary() const {
  auto str = this->str();
  if(entts.size() > 0) {
    str += " [";
    for(int i = 0; i < std::min(5, (int)entts.size()); i++) {
      str += entts[i]->name;
      if(i < std::min(5, (int)entts.size()) - 1) str += ",";
    }
    str += "]";
  }
  return str;
}

void Composition::resize(int32_t w, int32_t h) {
  size[0] = w;
  size[1] = h;
  if(frame_final) frame_final->resize(w, h);
  if(frame_edit) frame_edit->resize(w, h);
  if(frame_temp) frame_temp->resize(w, h);
}

Composition::Composition(const char* name, int32_t w, int32_t h, int32_t fps) {
  auto cmp = std::make_shared<Composition>();
  this->size[0] = w;
  this->size[1] = h;
  this->framerate_nu = fps;
  std::strncpy(cmp->name, name, MAX_DISPNAME);
  for(int i = 0; i < 10; i++) {
    TrackLayer layer;
    std::string name_str = "Layer" + std::to_string(i + 1);
    std::strncpy(layer.name, name_str.c_str(), MAX_DISPNAME);
    this->layers.push_back(layer);
  }
}

std::string Composition::str() const {
  std::string str = "Composition<" + std::string(name) + "/" + std::to_string(layers.size()) + ">";
  return str;
}

std::string Composition::summary() const {
  auto str = this->str();
  if(layers.size() > 0) {
    str += " [";
    for(int i = 0; i < std::min(5, (int)layers.size()); i++) {
      printf(" ------ layer %d\n", i);
      str += layers[i].summary();
      if(i < std::min(5, (int)layers.size()) - 1) str += "\n";
    }
    str += "]";
  }
  return str;
}

Composition* Composition::GetActiveComp() {
  return Project::GetActiveCompo();
}
} // namespace mu
