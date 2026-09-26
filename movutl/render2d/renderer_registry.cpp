#include <map>
#include <movutl/asset/config.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/render2d/renderer_registry.hpp>
#include <movutl/vulkan/vulkan_renderer.hpp>
#include <mutex>

namespace mu {

namespace {
struct Registry {
  std::mutex mtx;
  std::map<std::string, RendererFactory> factories;
  Registry() {
    factories["cpu"]    = [] { return std::make_unique<CPURenderer>(); };
    factories["vulkan"] = [] { return std::make_unique<VulkanRenderer>(); };
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
    LOG_F(WARNING, "Renderer '%s' is not registered. fallback to 'cpu'", name.c_str());
    it = r.factories.find("cpu");
  }
  return it->second();
}

const std::string& active_renderer_name() { return Config::Get()->renderer; }

// composition.cpp/render_worker.cpp/export_window.cppから毎フレーム・複数ワーカースレッドで呼ばれるためログは出さない(ログ出力はGUIでレンダラーを切り替えた箇所で行う)
std::unique_ptr<Renderer> create_active_renderer() { return create_renderer(active_renderer_name()); }

} // namespace mu
