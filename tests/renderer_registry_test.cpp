#include <cstdlib>
#include <doctest/doctest.h>
#include <fstream>
#include <movutl/asset/config.hpp>
#include <movutl/render2d/renderer_registry.hpp>

using namespace mu;

TEST_CASE("create_renderer: cpuは非null、未登録名はcpuにフォールバック") {
  CHECK(create_renderer("cpu") != nullptr);
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

TEST_CASE("resolve_renderer_name: cli > env > config > cpu") {
  CHECK(resolve_renderer_name("a", "b", "c") == "a");
  CHECK(resolve_renderer_name("", "b", "c") == "b");
  CHECK(resolve_renderer_name("", "", "c") == "c");
  CHECK(resolve_renderer_name("", "", "") == "cpu");
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

TEST_CASE("init_active_renderer: 環境変数がConfigを上書きする") {
  auto* c     = Config::Get();
  auto backup = c->renderer;
  c->renderer = "from_config";
  setenv("MOVUTL_RENDERER", "from_env", 1);
  init_active_renderer();
  CHECK(active_renderer_name() == "from_env");
  unsetenv("MOVUTL_RENDERER");
  init_active_renderer();
  CHECK(active_renderer_name() == "from_config");
  c->renderer = backup;
  init_active_renderer();
}
