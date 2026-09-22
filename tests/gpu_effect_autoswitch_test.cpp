#include <cstdlib>
#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/anim.hpp>
#include <movutl/render2d/renderer_registry.hpp>
#include <movutl/vulkan/vk_context.hpp>

using namespace mu;

namespace {
bool vk_ready() {
  if(VkContext::Get()->create()) return true;
  MESSAGE("SKIP: Vulkanデバイスが利用できません");
  return false;
}

void ensure_filters_registered() {
  static bool once = [] {
    if(detail::AppMain::Get()->filters.empty()) detail::register_default_filters();
    detail::activate_all_plugins();
    return true;
  }();
  (void)once;
}

FilterPluginTable* find_filter(const char* name) {
  for(auto& f : detail::AppMain::Get()->filters)
    if(std::string(f.name.c_str()) == name) return &f;
  return nullptr;
}
} // namespace

// active_renderer_name()を一時的に変更してrender_filtersの分岐を確認する。init_active_renderer()はMOVUTL_RENDERER環境変数を読む
TEST_CASE("Entity::render_filters: active_rendererでfn_proc/fn_proc_gpuが切り替わる") {
  if(!vk_ready()) return;
  ensure_filters_registered();
  FilterPluginTable* invert = find_filter("反転");
  REQUIRE(invert != nullptr);
  REQUIRE(invert->fn_proc_gpu != nullptr); // register_default_plugins.cppで配線済み

  auto comp = cutil::make_ref<Composition>("gpu_switch_comp", 4, 4, 30);
  auto img  = Image::Create("gpu_switch_test", 4, 4, ImageFormatRGBA, false);
  REQUIRE(img != nullptr);
  for(size_t i = 0; i < img->size(); i++) img->data()[i] = Vec4b(10, 20, 30, 255);

  FilterParam fp;
  fp.plg_ = invert;
  fp.props.add_props(invert->defaults);
  img->filters_.push_back(fp);

  const char* prev   = std::getenv("MOVUTL_RENDERER");
  std::string prev_s = prev ? prev : "";
  Image target(4, 4);

  // cpu: fn_proc(CPU反転)が使われる(comp->bg_color=0の透明背景にAlpha合成するとsrcの色がそのまま出る)
  setenv("MOVUTL_RENDERER", "cpu", 1);
  init_active_renderer();
  REQUIRE(img->render(comp.get(), &target, 0));
  CHECK(target.data()[0] == Vec4b(245, 235, 225, 255));

  // vulkan: fn_proc_gpu(GPU反転)が使われる。結果はCPU版と一致するはず
  setenv("MOVUTL_RENDERER", "vulkan", 1);
  init_active_renderer();
  target.fill_rgba(Vec4b(0, 0, 0, 0));
  REQUIRE(img->render(comp.get(), &target, 0));
  CHECK(target.data()[0] == Vec4b(245, 235, 225, 255));

  if(prev_s.empty())
    unsetenv("MOVUTL_RENDERER");
  else
    setenv("MOVUTL_RENDERER", prev_s.c_str(), 1);
  init_active_renderer();
}
