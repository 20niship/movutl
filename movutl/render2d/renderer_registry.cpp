#include <cstdlib>
#include <map>
#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/render2d/renderer_registry.hpp>
#include <mutex>

namespace mu {

namespace {
struct Registry {
  std::mutex mtx;
  std::map<std::string, RendererFactory> factories;
  std::string cli_override;
  std::string active = kDefaultRendererName;
  Registry() {
    factories[kDefaultRendererName] = [] { return std::make_unique<CPURenderer>(); };
  }
};
Registry& registry() {
  static Registry r;
  return r;
}
} // namespace

void register_renderer(const std::string& name, RendererFactory factory) {
  auto& r = registry();
  std::lock_guard<std::mutex> lock(r.mtx);
  r.factories[name] = std::move(factory);
}

std::vector<std::string> renderer_names() {
  auto& r = registry();
  std::lock_guard<std::mutex> lock(r.mtx);
  std::vector<std::string> names;
  for(auto& kv : r.factories) names.push_back(kv.first);
  return names;
}

std::unique_ptr<Renderer> create_renderer(const std::string& name) {
  auto& r = registry();
  std::lock_guard<std::mutex> lock(r.mtx);
  auto it = r.factories.find(name);
  if(it == r.factories.end()) {
    LOG_F(WARNING, "Renderer '%s' is not registered. fallback to '%s'", name.c_str(), kDefaultRendererName);
    it = r.factories.find(kDefaultRendererName);
  }
  return it->second();
}

std::string resolve_renderer_name(const std::string& cli, const std::string& env, const std::string& config) {
  if(!cli.empty()) return cli;
  if(!env.empty()) return env;
  if(!config.empty()) return config;
  return kDefaultRendererName;
}

void set_renderer_cli_override(const std::string& name) {
  auto& r = registry();
  std::lock_guard<std::mutex> lock(r.mtx);
  r.cli_override = name;
}

void init_active_renderer() {
  auto& r         = registry();
  const char* env = std::getenv("MOVUTL_RENDERER");
  std::string cli;
  {
    std::lock_guard<std::mutex> lock(r.mtx);
    cli = r.cli_override;
  }
  auto name = resolve_renderer_name(cli, env ? env : "", Config::Get()->renderer);
  std::lock_guard<std::mutex> lock(r.mtx);
  r.active = name;
  LOG_F(INFO, "Renderer: %s", name.c_str());
}

const std::string& active_renderer_name() { return registry().active; }

std::unique_ptr<Renderer> create_active_renderer() { return create_renderer(active_renderer_name()); }

} // namespace mu
