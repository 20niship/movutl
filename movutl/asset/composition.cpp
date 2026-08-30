#define NOMINMAX

#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/prop_types.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

Ref<Entity> TrackLayer::find_entt(uint32_t frame) const {
  for(auto& e : entts)
    if(e->visible(frame)) return e;
  return nullptr;
}


const cutil::PropInfo* TrackLayer::getPropsInfo() const { return nullptr; } // 手動実装のためpygenの生成対象外

cutil::Prop TrackLayer::getProps() const {
  cutil::Prop p;
  p.set<std::string>("name", name.c_str());
  p.set<bool>("active", active);
  return p;
}

void TrackLayer::setProps(const cutil::Prop& p) {
  name   = cutil::get_or<std::string>(p, "name", name.c_str());
  active = cutil::get_or<bool>(p, "active", active);
}

std::string TrackLayer::str() const {
  std::string str = "Layer<" + std::string(name.c_str()) + " / entt:" + std::to_string(entts.size()) + ">";
  return str;
}
std::string TrackLayer::summary() const {
  auto str = this->str();
  if(entts.size() > 0) {
    str += " [";
    for(int i = 0; i < std::min(5, (int)entts.size()); i++) {
      str += entts[i]->name.c_str();
      if(i < std::min(5, (int)entts.size()) - 1) str += ",";
    }
    str += "]";
  }
  return str;
}

void Composition::resize(int32_t w, int32_t h) {
  size[0] = w;
  size[1] = h;
}

Composition::Composition(const char* name, int32_t w, int32_t h, int32_t fps) {
  this->size[0]   = w;
  this->size[1]   = h;
  this->framerate = (float)fps;
  this->name      = name;
  for(int i = 0; i < 10; i++) {
    TrackLayer layer;
    std::string name_str = "Layer" + std::to_string(i + 1);
    layer.name           = name_str;
    this->layers.push_back(layer);
  }
}

std::string Composition::str() const {
  std::string str = "Composition<" + std::string(name.c_str()) + "/" + std::to_string(layers.size()) + ">";
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

Composition* Composition::GetActiveComp() { return Project::GetActiveCompo(); }

const cutil::PropInfo* Composition::getPropsInfo() const { return nullptr; } // 手動実装のためpygenの生成対象外

cutil::Prop Composition::getProps() const {
  cutil::Prop p;
  p.set<std::string>("name", name.c_str());
  p.set<Vec2>("size", Vec2(size));
  p.set<float>("framerate", framerate);
  p.set<int32_t>("bg_color", bg_color);
  p.set<int32_t>("fstart", fstart);
  p.set<int32_t>("fend", fend);
  p.set<int32_t>("frame", frame.load());
  return p;
}

void Composition::setProps(const cutil::Prop& p) {
  name      = cutil::get_or<std::string>(p, "name", name.c_str());
  size      = Vec2d(cutil::get_or<Vec2>(p, "size", Vec2(size)));
  framerate = cutil::get_or<float>(p, "framerate", framerate);
  bg_color  = cutil::get_or<int32_t>(p, "bg_color", bg_color);
  fstart    = cutil::get_or<int32_t>(p, "fstart", fstart);
  fend      = cutil::get_or<int32_t>(p, "fend", fend);
  frame.store(cutil::get_or<int32_t>(p, "frame", frame.load()));
}


int Composition::insertable_layer_index() const {
  for(int i = 0; i < layers.size(); i++) {
    if(layers[i].entts.size() == 0) return i;
  }
  return -1;
}

void Composition::insert_entity(Ref<Entity> entt, int layer) {
  {
    std::lock_guard<std::mutex> lock(mtx);
    if(layer < 0) layer = insertable_layer_index();
    if(layer < 0) {
      this->layers.push_back(TrackLayer());
      layer = this->layers.size() - 1;
    }
    MU_ASSERT(layer >= 0 && layer <= layers.size());
    this->layers[layer].entts.push_back(entt);
  }
  cache.invalidate_all();
}

std::vector<Ref<Entity>> Composition::get_all_entities() const {
  std::lock_guard<std::mutex> lock(mtx);
  std::vector<Ref<Entity>> out;
  for(auto& layer : layers) {
    if(!layer.active) continue;
    for(auto& e : layer.entts)
      if(e) out.push_back(e);
  }
  return out;
}

Ref<Image> Composition::render_current_frame_main_thread() {
  Ref<Image> out;
  if(cache.get(frame, &out)) return out;
  CPURenderer renderer;
  renderer.render_frame(this, frame, out);
  cache.insert(frame, out, frame);
  return out;
}

} // namespace mu
