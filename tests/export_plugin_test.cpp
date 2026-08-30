#include <cutil/prop.hpp>
#include <doctest/doctest.h>
#include <filesystem>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/render2d/renderer.hpp>

using namespace mu;
namespace fs = std::filesystem;

namespace {
OutputPluginTable* find_output_plugin(const char* name) {
  for(auto& p : detail::AppMain::Get()->output_plugins)
    if(std::string(p.name) == name) return &p;
  return nullptr;
}
} // namespace

TEST_CASE("Output Plugin: PNG/MP4がregister_output_plugin経由でAppMainへ登録される") {
  detail::init_external_plugins();
  CHECK(find_output_plugin("PNG Image/Sequence") != nullptr);
  CHECK(find_output_plugin("Video (FFmpeg)") != nullptr);
}

TEST_CASE("Output Plugin: PNG Exporterでフレーム範囲を連番書き出しできる") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("PNG Image/Sequence");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_png_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);
  const fs::path base = out_dir / "frame.png";

  void* handle = plg->fn_open(base.string().c_str(), 4, 4, 30.0f, plg->defaults);
  REQUIRE(handle != nullptr);

  Image img(4, 4);
  img.fill_rgba(Vec4b(10, 20, 30, 255));
  CHECK(plg->fn_write_frame(handle, &img, 0));
  CHECK(plg->fn_write_frame(handle, &img, 1));
  CHECK(plg->fn_close(handle));

  CHECK(fs::exists(out_dir / "frame_000000.png"));
  CHECK(fs::exists(out_dir / "frame_000001.png"));

  // 書き出した画像を読み直し、BGRA<->RGBAの変換が正しく往復することを確認する
  auto loaded = Image::Create("loaded", (out_dir / "frame_000000.png").string().c_str());
  REQUIRE(loaded->width == 4);
  REQUIRE(loaded->height == 4);
  CHECK((*loaded)[0] == Vec4b(10, 20, 30, 255));

  fs::remove_all(out_dir);
}

TEST_CASE("Output Plugin: Video Exporterはmov/aviなどmp4以外の拡張子でも自動でmuxerを選んで書き出せる") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("Video (FFmpeg)");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_video_ext_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);

  for(const char* ext : {"mov", "avi", "mkv"}) {
    const fs::path out_file = out_dir / (std::string("out.") + ext);
    void* handle            = plg->fn_open(out_file.string().c_str(), 16, 16, 30.0f, plg->defaults);
    REQUIRE(handle != nullptr);

    Image img(16, 16);
    img.fill_rgba(Vec4b(0, 0, 0, 255));
    CHECK(plg->fn_write_frame(handle, &img, 0));
    CHECK(plg->fn_close(handle));

    CHECK(fs::exists(out_file));
    CHECK(fs::file_size(out_file) > 0);
  }

  fs::remove_all(out_dir);
}

TEST_CASE("Output Plugin: MP4 Exporterで複数フレームをエンコードしてファイルが生成される") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("Video (FFmpeg)");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_video_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);
  const fs::path out_file = out_dir / "out.mp4";

  void* handle = plg->fn_open(out_file.string().c_str(), 16, 16, 30.0f, plg->defaults);
  REQUIRE(handle != nullptr);

  Image img(16, 16);
  for(int i = 0; i < 5; i++) {
    img.fill_rgba(Vec4b((uint8_t)(i * 40), 0, 0, 255));
    CHECK(plg->fn_write_frame(handle, &img, i));
  }
  CHECK(plg->fn_close(handle));

  CHECK(fs::exists(out_file));
  CHECK(fs::file_size(out_file) > 0);

  fs::remove_all(out_dir);
}

TEST_CASE("Output Plugin: CPURendererでレンダリングした結果をPNG Exporterへそのまま渡せる(6.1のRenderer再利用)") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("PNG Image/Sequence");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  Composition comp("ExportTestComp", 8, 8, 30);
  comp.bg_color = (int32_t)0xFFFFFFFFu; // RGBA(255,255,255,255) 相当の背景色

  CPURenderer renderer;
  Ref<Image> rendered;
  REQUIRE(renderer.render_frame(&comp, 0, rendered));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_png_render_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);
  void* handle = plg->fn_open((out_dir / "out.png").string().c_str(), 8, 8, 30.0f, plg->defaults);
  REQUIRE(handle != nullptr);
  CHECK(plg->fn_write_frame(handle, rendered.get(), 0));
  CHECK(plg->fn_close(handle));
  CHECK(fs::exists(out_dir / "out_000000.png"));

  fs::remove_all(out_dir);
}
