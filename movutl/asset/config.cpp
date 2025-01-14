#include <movutl/asset/config.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/core/props.hpp>

constexpr const char* kConfigFilename = "movutl_cnf.json";

namespace mu {

void Config::Load() {
  auto c = Config::Get();
  if(!fs_exists(kConfigFilename)) {
    Save();
    return;
  }

  auto js = Props::LoadJsonFile(kConfigFilename);
  c->max_size = Vec2d(js.get_or<Vec2>("max_size", Vec2(c->max_size)));
  c->max_frame = js.get_or<int>("max_frame", c->max_frame);
  c->cache_frames = js.get_or<int>("cache_frames", c->cache_frames);
  /*c->plugin_search_paths = js.get_or<std::vector<std::string>>("plugin_search_paths", c->plugin_search_paths);*/
  c->log_to_file = js.get_or<bool>("log_to_file", c->log_to_file);
  c->log_filename = js.get_or<std::string>("log_filename", c->log_filename);
  c->log_level = (LogLevel)js.get_or<int>("log_level", int(c->log_level));
}

void Config::Save() {
  auto c = Config::Get();
  Props js;
  js.set("max_size", c->max_size);
  js.set("max_frame", c->max_frame);
  js.set("cache_frames", c->cache_frames);
  /*js.set("plugin_search_paths", c->plugin_search_paths);*/
  js.set("log_to_file", c->log_to_file);
  js.set("log_filename", c->log_filename);
  js.set("log_level", int(c->log_level));
  js.dump_json_file(kConfigFilename);
}

void Config::Reload() {
  Config::Save();
  Config::Load();
}

Config* Config::singleton_ = nullptr;
} // namespace mu
