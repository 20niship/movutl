#include <movutl/asset/composition.hpp>

namespace mu {
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
    std::string name_str = "Layer " + std::to_string(i + 1);
    std::strncpy(layer.name, name_str.c_str(), MAX_DISPNAME);
    this->layers.push_back(layer);
  }
}

} // namespace mu
