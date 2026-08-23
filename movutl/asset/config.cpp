#include <cutil/prop_io.hpp>
#include <fstream>
#include <movutl/asset/config.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/prop_types.hpp>
#include <sstream>

constexpr const char* kConfigFilename = "movutl_cnf.json";

namespace mu {

void Config::Load() {
  auto c = Config::Get();
  if(!fs_exists(kConfigFilename)) {
    Save();
    return;
  }

  std::ifstream ifs(kConfigFilename);
  std::stringstream ss;
  ss << ifs.rdbuf();
  cutil::Prop js;
  cutil::prop_load_json(js, ss.str());

  c->max_size     = Vec2d(cutil::get_or<Vec2>(js, "max_size", Vec2(c->max_size)));
  c->max_frame    = cutil::get_or<int32_t>(js, "max_frame", c->max_frame);
  c->cache_frames = cutil::get_or<int32_t>(js, "cache_frames", c->cache_frames);
  c->log_to_file  = cutil::get_or<bool>(js, "log_to_file", c->log_to_file);
  c->log_filename = cutil::get_or<std::string>(js, "log_filename", c->log_filename);
  c->log_level    = (LogLevel)cutil::get_or<int32_t>(js, "log_level", int(c->log_level));
  c->show_viewer_ruler = cutil::get_or<bool>(js, "show_viewer_ruler", c->show_viewer_ruler);
}

void Config::Save() {
  auto c = Config::Get();
  cutil::Prop js;
  js.set<Vec2>("max_size", Vec2(c->max_size));
  js.set<int32_t>("max_frame", c->max_frame);
  js.set<int32_t>("cache_frames", c->cache_frames);
  js.set<bool>("log_to_file", c->log_to_file);
  js.set<std::string>("log_filename", c->log_filename);
  js.set<int32_t>("log_level", int(c->log_level));
  js.set<bool>("show_viewer_ruler", c->show_viewer_ruler);

  std::string out;
  cutil::prop_dump_json(js, out);
  std::ofstream ofs(kConfigFilename);
  ofs << out;
}

void Config::Reload() {
  Config::Save();
  Config::Load();
}

Config* Config::singleton_ = nullptr;
} // namespace mu
