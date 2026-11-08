#include <cstring>
#include <movutl/paint/stroke.hpp>
#include <movutl/paint/stroke_registry.hpp>
#include <movutl/paint/strokes.hpp>
#include <movutl/raster/blend.hpp>

namespace mu {

Selection Selection::rect(int w, int h, int x0, int y0, int x1, int y1) {
  Selection s;
  s.w = w;
  s.h = h;
  s.m.assign((size_t)w * h, 0);
  for(int y = std::max(0, y0); y < std::min(h, y1); y++)
    for(int x = std::max(0, x0); x < std::min(w, x1); x++) s.m[(size_t)y * w + x] = 255;
  return s;
}

Stroke::Stroke(TiledImage* target, const BrushSettings& s) : target_(target), settings_(s), rng_(s.seed ? s.seed : 1) {
  tip_ = s.tip;
  if(!tip_ && !s.tip_name.empty()) tip_ = TipRegistry::Get().create(s.tip_name, s.params);
  if(!tip_) tip_ = std::make_shared<CircleTip>(s.hardness);
}

float Stroke::rnd01() {
  rng_ ^= rng_ << 13; // xorshift32
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return (rng_ & 0xFFFFFF) / (float)0x1000000;
}

StampInfo Stroke::stamp_info(const PenSample& s, float dir_angle, float flow_mul) {
  const float pr = std::clamp(s.pressure, 0.0f, 1.0f);
  float size_mul = settings_.pressure_size ? std::max(pr, 0.05f) : 1.0f;
  if(settings_.size_jitter > 0) size_mul *= 1.0f - settings_.size_jitter * rnd01();
  StampInfo i;
  i.radius   = 0.5f * settings_.size * size_mul;
  i.flow     = settings_.opacity * (settings_.pressure_opacity ? pr : 1.0f) * flow_mul;
  i.pressure = pr;
  i.angle    = settings_.angle + (settings_.angle_follow ? dir_angle : 0.0f);
  i.x        = s.x;
  i.y        = s.y;
  if(settings_.scatter > 0) {
    const float r = settings_.scatter * settings_.size * std::sqrt(rnd01()), a = rnd01() * 6.2831853f;
    i.x += r * std::cos(a);
    i.y += r * std::sin(a);
  }
  return i;
}

bool Stroke::make_stamp(const StampInfo& i, Stamp& out) {
  if(i.radius <= 0.0f || i.flow <= 0.0f) return false;
  const int x0 = std::max(0, (int)std::floor(i.x - i.radius) - 1), x1 = std::min(target_->width() - 1, (int)std::ceil(i.x + i.radius) + 1);
  const int y0 = std::max(0, (int)std::floor(i.y - i.radius) - 1), y1 = std::min(target_->height() - 1, (int)std::ceil(i.y + i.radius) + 1);
  if(x1 < x0 || y1 < y0) return false;
  out.x0 = x0;
  out.y0 = y0;
  out.w  = x1 - x0 + 1;
  out.h  = y1 - y0 + 1;
  out.coverage.assign((size_t)out.w * out.h, 0.0f);
  tip_->render({i.radius, i.angle, i.x - x0, i.y - y0}, out.w, out.h, out.coverage.data());
  const bool tex = settings_.texture.img != nullptr;
  if(tex || clip_) {
    for(int j = 0; j < out.h; j++) {
      for(int k = 0; k < out.w; k++) {
        float& c = out.coverage[(size_t)j * out.w + k];
        if(c <= 0.0f) continue;
        if(tex) c *= settings_.texture.sample(x0 + k, y0 + j);
        if(clip_) c *= clip_->at(x0 + k, y0 + j);
      }
    }
  }
  return true;
}

// ---- StrokeRegistry / TipRegistry ----
namespace {
template <class T> StrokePluginTable builtin(const char* name, const char* label, const char* info) {
  StrokePluginTable t;
  std::strncpy(t.name, name, sizeof(t.name) - 1);
  std::strncpy(t.label, label, sizeof(t.label) - 1);
  std::strncpy(t.information, info, sizeof(t.information) - 1);
  t.create = [](TiledImage* i, const BrushSettings& s) -> Stroke* { return new T(i, s); };
  return t;
}
} // namespace

StrokeRegistry::StrokeRegistry() {
  register_type(builtin<PaintStroke>("paint", "ペン", "色を重ねて描く標準のブラシ"));
  register_type(builtin<EraseStroke>("erase", "消しゴム", "アルファを減らして消す"));
  register_type(builtin<AirbrushStroke>("airbrush", "エアブラシ", "止めている間も濃さが積もる"));
  register_type(builtin<PatternStroke>("pattern", "テクスチャ貼り", "パターン画像をなぞって貼る"));
  register_type(builtin<ToneStroke>("tone", "トーン", "濃さを網点で表現する"));
  register_type(builtin<ParticleStroke>("particle", "パーティクル", "小さな粒を撒く"));
  register_type(builtin<WatercolorStroke>("watercolor", "水彩", "下地と混ざり縁が濃くなる"));
}

StrokeRegistry& StrokeRegistry::Get() {
  static StrokeRegistry r;
  return r;
}

bool StrokeRegistry::register_type(const StrokePluginTable& t) {
  if(t.name[0] == '\0' || !t.create) return false;
  types_[t.name] = t;
  return true;
}

const StrokePluginTable* StrokeRegistry::find(const std::string& name) const {
  auto it = types_.find(name);
  return it == types_.end() ? nullptr : &it->second;
}

std::unique_ptr<Stroke> StrokeRegistry::create(const std::string& name, TiledImage* target, const BrushSettings& s) const {
  auto* t = find(name);
  if(!t) return nullptr;
  return std::unique_ptr<Stroke>(t->create(target, s)); // paramsが未設定の項目はプラグイン側がcutil::get_orの既定値で読む
}

std::vector<std::string> StrokeRegistry::list() const {
  std::vector<std::string> v;
  for(auto& [k, _] : types_) v.push_back(k);
  return v;
}

TipRegistry::TipRegistry() {
  TipPluginTable t;
  std::strncpy(t.name, "circle", sizeof(t.name) - 1);
  std::strncpy(t.label, "円形", sizeof(t.label) - 1);
  t.create = [](const cutil::Prop&) -> std::shared_ptr<BrushTip> { return std::make_shared<CircleTip>(); };
  register_type(t);
}

TipRegistry& TipRegistry::Get() {
  static TipRegistry r;
  return r;
}

bool TipRegistry::register_type(const TipPluginTable& t) {
  if(t.name[0] == '\0' || !t.create) return false;
  types_[t.name] = t;
  return true;
}

std::shared_ptr<BrushTip> TipRegistry::create(const std::string& name, const cutil::Prop& params) const {
  auto it = types_.find(name);
  return it == types_.end() ? nullptr : it->second.create(params);
}

std::vector<std::string> TipRegistry::list() const {
  std::vector<std::string> v;
  for(auto& [k, _] : types_) v.push_back(k);
  return v;
}

} // namespace mu
