#include <cutil/prop_io.hpp>
#include <fstream>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/prop_types.hpp>
#include <sstream>

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
  auto pj           = Project::Get();
  pj->main_comp_idx = idx;
}

void Project::Save(const char* path) {
  auto pj      = Project::Get();
  auto save_to = path ? std::string(path) : pj->path;
  MU_ASSERT(!save_to.empty());

  cutil::Prop js;
  js.set<std::string>("output_path", pj->output_path);
  js.set<int32_t>("main_comp_idx", pj->main_comp_idx);

  js.set<int32_t>("entity_count", (int32_t)pj->entities.size());
  for(size_t i = 0; i < pj->entities.size(); i++) js.set_child(("entity_" + std::to_string(i)).c_str(), pj->entities[i]->getSaveProps());

  js.set<int32_t>("compo_count", (int32_t)pj->compos_.size());
  for(size_t ci = 0; ci < pj->compos_.size(); ci++) {
    const auto& cmp = pj->compos_[ci];
    cutil::Prop cp;
    cp.set_child("props", cmp.getProps());

    cp.set<int32_t>("layer_count", (int32_t)cmp.layers.size());
    for(size_t li = 0; li < cmp.layers.size(); li++) {
      const auto& layer = cmp.layers[li];
      cutil::Prop lp;
      lp.set_child("props", layer.getProps());
      lp.set<int32_t>("entt_count", (int32_t)layer.entts.size());
      for(size_t ei = 0; ei < layer.entts.size(); ei++) lp.set<int32_t>(("entt_guid_" + std::to_string(ei)).c_str(), (int32_t)layer.entts[ei]->guid_);
      cp.set_child(("layer_" + std::to_string(li)).c_str(), lp);
    }
    js.set_child(("compo_" + std::to_string(ci)).c_str(), cp);
  }

  std::string out;
  cutil::prop_dump_json(js, out);
  std::ofstream ofs(save_to);
  ofs << out;
  pj->path = save_to;
}

void Project::Load(const char* path) {
  MU_ASSERT(path != nullptr);
  auto pj = Project::Get();
  pj->compos_.clear();
  pj->entities.clear();

  std::ifstream ifs(path);
  std::stringstream ss;
  ss << ifs.rdbuf();
  cutil::Prop js;
  cutil::prop_load_json(js, ss.str());

  pj->path          = path;
  pj->output_path   = cutil::get_or<std::string>(js, "output_path", "");
  pj->main_comp_idx = cutil::get_or<int32_t>(js, "main_comp_idx", 0);

  int32_t entity_count = cutil::get_or<int32_t>(js, "entity_count", 0);
  for(int32_t i = 0; i < entity_count; i++) Entity::fromSaveProps(js.get_child(("entity_" + std::to_string(i)).c_str()));

  int32_t compo_count = cutil::get_or<int32_t>(js, "compo_count", 0);
  for(int32_t ci = 0; ci < compo_count; ci++) {
    const auto& cp = js.get_child(("compo_" + std::to_string(ci)).c_str());
    Composition cmp;
    if(cp.contains("props")) cmp.setProps(cp.get_child("props"));

    int32_t layer_count = cutil::get_or<int32_t>(cp, "layer_count", 0);
    for(int32_t li = 0; li < layer_count; li++) {
      const auto& lp = cp.get_child(("layer_" + std::to_string(li)).c_str());
      TrackLayer layer;
      if(lp.contains("props")) layer.setProps(lp.get_child("props"));

      int32_t entt_count = cutil::get_or<int32_t>(lp, "entt_count", 0);
      for(int32_t ei = 0; ei < entt_count; ei++) {
        int32_t guid = cutil::get_or<int32_t>(lp, ("entt_guid_" + std::to_string(ei)).c_str(), 0);
        for(auto& e : pj->entities) {
          if(e->guid_ == guid) {
            layer.entts.push_back(e);
            break;
          }
        }
      }
      cmp.layers.push_back(layer);
    }
    pj->compos_.push_back(cmp);
  }
}

Project* Project::singleton_ = nullptr;

} // namespace mu
