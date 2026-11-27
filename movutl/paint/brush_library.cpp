#include <algorithm>
#include <cutil/prop_io.hpp>
#include <filesystem>
#include <fstream>
#include <movutl/core/filesystem.hpp>
#include <movutl/paint/brush_library.hpp>
#include <nlohmann/json.hpp>

namespace mu {

namespace {

// 保存するBrushSettingsの項目。dumpとloadで共有する
template <class S, class F> void for_each_field(S& s, F&& f) {
  f("engine", s.engine);
  f("size", s.size);
  f("hardness", s.hardness);
  f("opacity", s.opacity);
  f("spacing", s.spacing);
  f("pressure_size", s.pressure_size);
  f("pressure_opacity", s.pressure_opacity);
  f("tip_name", s.tip_name);
  f("texture_scale", s.texture.scale);
  f("texture_depth", s.texture.depth);
  f("angle", s.angle);
  f("angle_follow", s.angle_follow);
  f("size_jitter", s.size_jitter);
  f("scatter", s.scatter);
  f("seed", s.seed);
  f("stabilization", s.stabilization);
  f("airbrush_rate", s.airbrush_rate);
  f("pattern_scale", s.pattern_scale);
  f("tone_pitch", s.tone_pitch);
  f("tone_angle", s.tone_angle);
  f("particle_count", s.particle_count);
  f("particle_size", s.particle_size);
  f("water_mix", s.water_mix);
  f("edge_darkening", s.edge_darkening);
}

nlohmann::json to_json(const BrushSettings& s) {
  nlohmann::json j;
  for_each_field(s, [&](const char* k, const auto& v) { j[k] = v; });
  std::string params;
  if(cutil::prop_dump_json(s.params, params)) j["params"] = params;
  return j;
}

BrushSettings from_json(const nlohmann::json& j) {
  BrushSettings s;
  for_each_field(s, [&](const char* k, auto& v) { v = j.value(k, v); }); // 無い項目は既定値
  const std::string params = j.value("params", std::string());
  if(!params.empty()) cutil::prop_load_json(s.params, params);
  return s;
}

void load_pattern(BrushPreset& p) {
  if(p.pattern_file.empty()) return;
  p.initial.pattern = p.current.pattern = load_brush_image(fs_get_asset_path() + "/brushes/" + p.pattern_file);
}

} // namespace

BrushLibrary BrushLibrary::standard() {
  BrushLibrary lib;
  auto add = [&](const char* tool, const char* group, const char* name, const char* engine, float size, float hardness, float opacity) -> BrushPreset& {
    BrushPreset p;
    p.name             = name;
    p.group            = group;
    p.tool             = tool;
    p.initial.engine   = engine;
    p.initial.size     = size;
    p.initial.hardness = hardness;
    p.initial.opacity  = opacity;
    p.reset();
    lib.presets.push_back(std::move(p));
    return lib.presets.back();
  };
  // 初期値と現在値の両方に反映する
  auto both = [](BrushPreset& p, auto&& f) {
    f(p.initial);
    p.reset();
  };

  add("pen", "ペン", "Gペン", "paint", 10, 1.0f, 1.0f);
  both(add("pencil", "鉛筆", "鉛筆", "paint", 4, 0.6f, 0.8f), [](BrushSettings& s) {
    s.pressure_size    = false;
    s.pressure_opacity = true;
  });
  add("brush", "水彩", "水彩", "watercolor", 40, 0.6f, 0.8f);
  both(add("airbrush", "エアブラシ", "柔らか", "airbrush", 80, 0.0f, 0.15f), [](BrushSettings& s) { s.pressure_size = false; });
  both(add("airbrush", "エアブラシ", "スプレー", "particle", 60, 0.8f, 1.0f), [](BrushSettings& s) {
    s.spacing       = 0.3f;
    s.particle_size = 0.1f;
  });
  both(add("deco", "トーン", "トーン", "tone", 40, 0.8f, 0.5f), [](BrushSettings& s) { s.pressure_size = false; });
  {
    auto& p        = add("deco", "パターン", "パターン", "pattern", 40, 0.8f, 1.0f);
    p.pattern_file = "pattern_dots.png";
    load_pattern(p);
  }
  both(add("eraser", "消しゴム", "硬め", "erase", 30, 1.0f, 1.0f), [](BrushSettings& s) { s.pressure_size = false; });
  return lib;
}

std::vector<BrushPreset*> BrushLibrary::of_tool(const std::string& tool) {
  std::vector<BrushPreset*> v;
  for(auto& p : presets)
    if(p.tool == tool) v.push_back(&p);
  return v;
}

BrushPreset* BrushLibrary::selected(const std::string& tool) {
  const auto v = of_tool(tool);
  if(v.empty()) return nullptr;
  const auto it = selected_.find(tool);
  if(it != selected_.end())
    for(auto* p : v)
      if(p->name == it->second) return p;
  return v.front();
}

bool BrushLibrary::select(const std::string& tool, const std::string& name) {
  for(auto* p : of_tool(tool)) {
    if(p->name != name) continue;
    selected_[tool] = name;
    return true;
  }
  return false;
}

bool BrushLibrary::duplicate(const std::string& tool, const std::string& name) {
  const auto v = of_tool(tool);
  for(auto* p : v) {
    if(p->name != name) continue;
    BrushPreset copy = *p;
    // 名前で選択を覚えるので、ツール内で重複しないようにする
    for(int n = 1;; n++) {
      copy.name = name + " のコピー" + (n > 1 ? " " + std::to_string(n) : "");
      if(std::none_of(v.begin(), v.end(), [&](const BrushPreset* o) { return o->name == copy.name; })) break;
    }
    selected_[tool] = copy.name;
    presets.insert(presets.begin() + (p - presets.data()) + 1, std::move(copy));
    return true;
  }
  return false;
}

bool BrushLibrary::remove(const std::string& tool, const std::string& name) {
  const auto v = of_tool(tool);
  for(size_t i = 0; i < v.size(); i++) {
    if(v[i]->name != name || v.size() <= 1) continue;
    if(selected(tool) == v[i]) selected_[tool] = v[i + 1 < v.size() ? i + 1 : i - 1]->name;
    presets.erase(presets.begin() + (v[i] - presets.data()));
    return true;
  }
  return false;
}

bool BrushLibrary::move(const std::string& tool, const std::string& name, int dir) {
  auto v = of_tool(tool);
  for(auto* p : v) {
    if(p->name != name) continue;
    std::erase_if(v, [&](const BrushPreset* o) { return o->group != p->group; });
    const auto i = std::find(v.begin(), v.end(), p) - v.begin() + (dir < 0 ? -1 : 1);
    if(dir == 0 || i < 0 || i >= (ptrdiff_t)v.size()) return false;
    if(selected_.count(tool) == 0) selected_[tool] = selected(tool)->name; // 未選択(先頭)のまま並びが変わると選択が移る
    std::swap(*p, *v[i]);
    return true;
  }
  return false;
}

std::string BrushLibrary::dump() const {
  nlohmann::json j;
  j["version"]  = 1;
  j["selected"] = selected_;
  j["presets"]  = nlohmann::json::array();
  for(const auto& p : presets) j["presets"].push_back({{"name", p.name}, {"group", p.group}, {"tool", p.tool}, {"pattern_file", p.pattern_file}, {"initial", to_json(p.initial)}, {"current", to_json(p.current)}});
  return j.dump(2);
}

bool BrushLibrary::save(const std::string& path) const {
  const std::string tmp = path + ".tmp"; // 書き込みに失敗しても既存のファイルを壊さない
  std::ofstream f(tmp, std::ios::binary);
  if(!f) return false;
  f << dump();
  f.close();
  std::error_code ec;
  if(f) std::filesystem::rename(tmp, path, ec);
  if(f && !ec) return true;
  std::filesystem::remove(tmp, ec);
  return false;
}

bool BrushLibrary::load(const std::string& path) try {
  std::ifstream f(path, std::ios::binary);
  if(!f) return false;
  const auto j = nlohmann::json::parse(f);
  std::vector<BrushPreset> loaded;
  for(const auto& e : j.at("presets")) {
    BrushPreset p;
    p.name         = e.at("name").get<std::string>();
    p.tool         = e.at("tool").get<std::string>();
    p.group        = e.value("group", std::string());
    p.pattern_file = e.value("pattern_file", std::string());
    p.initial      = from_json(e.at("initial"));
    p.current      = from_json(e.at("current"));
    load_pattern(p);
    loaded.push_back(std::move(p));
  }
  auto selected = j.value("selected", std::map<std::string, std::string>());
  presets       = std::move(loaded);
  selected_     = std::move(selected);
  return true;
} catch(const std::exception&) { // jsonの型違いや、paramsの壊れた値
  return false;
}

} // namespace mu
