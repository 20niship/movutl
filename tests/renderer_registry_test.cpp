#include <doctest/doctest.h>
#include <movutl/asset/config.hpp>
#include <movutl/render2d/renderer_registry.hpp>

using namespace mu;

TEST_CASE("create_renderer: cpu/vulkanが非null、未登録名はcpuにフォールバック") {
  CHECK(create_renderer("cpu") != nullptr);
  CHECK(create_renderer("vulkan") != nullptr); // Vulkan不可環境ではVulkanRenderer内部でCPURendererへ委譲する
  auto r = create_renderer("no_such_renderer");
  REQUIRE(r != nullptr);
  CHECK(dynamic_cast<CPURenderer*>(r.get()) != nullptr);
}

TEST_CASE("register_renderer: 登録した名前で生成できる") {
  register_renderer("test_dummy", [] { return std::make_unique<CPURenderer>(); });
  auto names = renderer_names();
  CHECK(std::find(names.begin(), names.end(), "test_dummy") != names.end());
  CHECK(create_renderer("test_dummy") != nullptr);
}

TEST_CASE("Config: rendererが保存/読込で保持される") {
  auto* c     = Config::Get();
  auto backup = c->renderer;
  c->renderer = "vulkan";
  Config::Save();
  c->renderer = "cpu";
  Config::Load();
  CHECK(c->renderer == "vulkan");
  c->renderer = backup;
  Config::Save();
}

TEST_CASE("active_renderer_name: Config::Get()->rendererをその場で反映する(再起動不要)") {
  auto* c     = Config::Get();
  auto backup = c->renderer;
  c->renderer = "vulkan";
  CHECK(active_renderer_name() == "vulkan");
  c->renderer = "cpu";
  CHECK(active_renderer_name() == "cpu");
  c->renderer = backup;
}
