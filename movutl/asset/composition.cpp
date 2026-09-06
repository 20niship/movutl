#define NOMINMAX

#include <algorithm>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/prop_types.hpp>
#include <movutl/render2d/renderer.hpp>

namespace mu {

namespace {
thread_local std::vector<uint32_t> g_render_stack;
std::atomic<uint32_t> g_next_compo_guid{1}; // 0はtarget_comp_guid未設定を表す予約値
} // namespace

bool Composition::PushRenderGuard(uint32_t guid) {
  if(std::find(g_render_stack.begin(), g_render_stack.end(), guid) != g_render_stack.end()) {
    LOG_F(WARNING, "Composition: circular reference detected (guid=%u), skipping", guid);
    return false;
  }
  g_render_stack.push_back(guid);
  return true;
}

void Composition::PopRenderGuard(uint32_t guid) {
  MU_ASSERT(!g_render_stack.empty() && g_render_stack.back() == guid);
  g_render_stack.pop_back();
}

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

Composition::Composition() {
  guid      = g_next_compo_guid++;
  audio_buf = cutil::make_ref<AudioRingBuffer>(audio_sample_rate, audio_channels);
}

Composition::Composition(const char* name, int32_t w, int32_t h, int32_t fps) {
  guid            = g_next_compo_guid++;
  this->size[0]   = w;
  this->size[1]   = h;
  this->framerate = (float)fps;
  this->name      = name;
  for(int i = 0; i < 10; i++) {
    TrackLayer layer;
    std::string name_str = "レイヤー" + std::to_string(i + 1);
    layer.name           = name_str;
    this->layers.push_back(layer);
  }
  audio_buf = cutil::make_ref<AudioRingBuffer>(audio_sample_rate, audio_channels);
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
  p.set<int32_t>("guid", (int32_t)guid);
  p.set<std::string>("name", name.c_str());
  p.set<Vec2>("size", Vec2(size));
  p.set<float>("framerate", framerate);
  p.set<int32_t>("bg_color", bg_color);
  p.set<int32_t>("fstart", fstart);
  p.set<int32_t>("fend", fend);
  p.set<int32_t>("frame", frame.load());
  p.set<int32_t>("audio_sample_rate", audio_sample_rate);
  p.set<int32_t>("audio_channels", audio_channels);
  return p;
}

void Composition::setProps(const cutil::Prop& p) {
  // ロードしたguidがCompoRefEntt/CompoAudioEntt::target_comp_guidの参照先解決に必要なため復元し、以後の新規Compositionと衝突しないようカウンタを追い越す
  guid = (uint32_t)cutil::get_or<int32_t>(p, "guid", (int32_t)guid);
  if(guid >= g_next_compo_guid) g_next_compo_guid = guid + 1;
  name      = cutil::get_or<std::string>(p, "name", name.c_str());
  size      = Vec2d(cutil::get_or<Vec2>(p, "size", Vec2(size)));
  framerate = cutil::get_or<float>(p, "framerate", framerate);
  bg_color  = cutil::get_or<int32_t>(p, "bg_color", bg_color);
  fstart    = cutil::get_or<int32_t>(p, "fstart", fstart);
  fend      = cutil::get_or<int32_t>(p, "fend", fend);
  frame.store(cutil::get_or<int32_t>(p, "frame", frame.load()));
  audio_sample_rate = cutil::get_or<int32_t>(p, "audio_sample_rate", audio_sample_rate);
  audio_channels    = cutil::get_or<int32_t>(p, "audio_channels", audio_channels);
  audio_buf         = cutil::make_ref<AudioRingBuffer>(audio_sample_rate, audio_channels);
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
  invalidate_cache_all();
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

Ref<Image> Composition::render_current_frame_main_thread(bool transparent_bg) {
  FrameCache& c = transparent_bg ? cache_transparent : cache;
  Ref<Image> out;
  if(c.get(frame, &out)) return out;
  if(!PushRenderGuard(guid)) return nullptr;
  CPURenderer renderer;
  renderer.render_frame(this, frame, out, transparent_bg);
  c.insert(frame, out, frame);
  PopRenderGuard(guid);
  return out;
}

void Composition::invalidate_cache_all() {
  cache.invalidate_all();
  cache_transparent.invalidate_all();
  // ponytail: Composition数が多くなると毎回全探索は重くなる。逆参照インデックスを持つ設計に変える余地あり
  for(auto& c : Project::Get()->compos_) {
    if(c->guid == guid) continue;
    for(auto& e : c->get_all_entities()) {
      uint32_t target = 0;
      if(e->getType() == EntityType_Scene)
        target = static_cast<CompoRefEntt*>(e.get())->target_comp_guid;
      else if(e->getType() == EntityType_SceneAudio)
        target = static_cast<CompoAudioEntt*>(e.get())->target_comp_guid;
      else
        continue;
      if(target == guid) {
        c->cache.invalidate_all();
        c->cache_transparent.invalidate_all();
        break;
      }
    }
  }
}

void Composition::invalidate_cache_range(int f0, int f1) {
  cache.invalidate_range(f0, f1);
  cache_transparent.invalidate_range(f0, f1);
  // ponytail: 伝播先での正確な範囲変換(速度/開始フレームずれ)が煩雑なため安全側で全体無効化する
  for(auto& c : Project::Get()->compos_) {
    if(c->guid == guid) continue;
    for(auto& e : c->get_all_entities()) {
      uint32_t target = 0;
      if(e->getType() == EntityType_Scene)
        target = static_cast<CompoRefEntt*>(e.get())->target_comp_guid;
      else if(e->getType() == EntityType_SceneAudio)
        target = static_cast<CompoAudioEntt*>(e.get())->target_comp_guid;
      else
        continue;
      if(target == guid) {
        c->cache.invalidate_all();
        c->cache_transparent.invalidate_all();
        break;
      }
    }
  }
}

} // namespace mu
