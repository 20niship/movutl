#include <cstdlib>
#include <cstring>
#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/plugin/input.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;

namespace {

/// テスト用にplugins/以下の.msoプラグイン(FFmpeg video reader等)を読み込む
struct PluginRegister {
  PluginRegister() {
    detail::register_default_plugins();
    detail::init_external_plugins();
  }
};

static PluginRegister plugin_register_;

/// テスト用の小さな動画をffmpeg CLIで生成する (testsrc / smptebars)
std::string gen_test_video(const std::string& filename, const std::string& lavfi_src) {
  const std::string path = "/tmp/opencode/" + filename;
  if(fs_exists(path)) return path;
  const std::string cmd = "ffmpeg -y -loglevel error -f lavfi -i " + lavfi_src + " -pix_fmt yuv420p -c:v libx264 " + path;
  const int r           = std::system(cmd.c_str());
  REQUIRE_MESSAGE(r == 0, "failed to generate test video: " << cmd);
  return path;
}

/// 動画から指定フレームを読み込んでImageへ格納する
bool read_frame(Movie* mov, int frame, Image* img) {
  img->resize(mov->get_info().width, mov->get_info().height);
  auto plg = mov->get_input_plugin();
  return plg && plg->fn_read_video && plg->fn_read_video(mov->get_input_handle(), frame, img->data()) > 0;
}

bool same_image(Image* a, Image* b) { return a->width == b->width && a->height == b->height && std::memcmp(a->data(), b->data(), (size_t)a->width * a->height * 4) == 0; }

} // namespace

TEST_CASE("FFmpeg Video Reader: ファイル情報取得") {
  Movie mov;
  REQUIRE(mov.load_file(gen_test_video("mvtest_a.mp4", "testsrc=duration=2:size=160x120:rate=10").c_str()));
  CHECK(mov.get_info().width == 160);
  CHECK(mov.get_info().height == 120);
  CHECK(mov.get_info().nframes == 20);
  CHECK(std::abs(mov.get_info().framerate - 10.0f) < 0.5f);
  CHECK(mov.trk.fend == (int)mov.get_info().nframes);

  /// 存在しないファイル
  {
    Movie m;
    CHECK_FALSE(m.load_file("/nonexistent/video.mp4"));
  }
  /// 動画でないファイル
  {
    FILE* f = fopen("/tmp/opencode/mvtest_notavideo.mp4", "w");
    fputs("this is not a video", f);
    fclose(f);
    Movie m;
    CHECK_FALSE(m.load_file("/tmp/opencode/mvtest_notavideo.mp4"));
  }
}

TEST_CASE("FFmpeg Video Reader: シークと決定性") {
  Movie mov;
  REQUIRE(mov.load_file(gen_test_video("mvtest_a.mp4", "testsrc=duration=2:size=160x120:rate=10").c_str()));

  Image f0a, f3a, f7a, f0b, f3b, f7b;
  /// 順方向に読む
  REQUIRE(read_frame(&mov, 0, &f0a));
  REQUIRE(read_frame(&mov, 3, &f3a));
  REQUIRE(read_frame(&mov, 7, &f7a));
  /// ランダムアクセス後に再読み込み → 同じ内容になるはず
  REQUIRE(read_frame(&mov, 3, &f3b));
  CHECK(same_image(&f3a, &f3b));
  REQUIRE(read_frame(&mov, 7, &f7b));
  CHECK(same_image(&f7a, &f7b));
  /// 先頭への逆シーク
  REQUIRE(read_frame(&mov, 0, &f0b));
  CHECK(same_image(&f0a, &f0b));
}

TEST_CASE("FFmpeg Video Reader: 複数動画の同時読み込み") {
  const std::string path_a = gen_test_video("mvtest_a.mp4", "testsrc=duration=2:size=160x120:rate=10");
  const std::string path_b = gen_test_video("mvtest_b.mp4", "smptebars=duration=2:size=320x240:rate=10");

  /// 単体での参照読み込み
  Movie ref_a, ref_b;
  REQUIRE(ref_a.load_file(path_a.c_str()));
  REQUIRE(ref_b.load_file(path_b.c_str()));
  Image ra3, rb5;
  REQUIRE(read_frame(&ref_a, 3, &ra3));
  REQUIRE(read_frame(&ref_b, 5, &rb5));

  /// 同時オープンして交互に読み込む → 混線しないこと
  Movie ma, mb;
  REQUIRE(ma.load_file(path_a.c_str()));
  REQUIRE(mb.load_file(path_b.c_str()));
  Image ia, ib;
  for(int i = 0; i < 4; i++) {
    REQUIRE(read_frame(&ma, 3 + i, &ia));
    REQUIRE(read_frame(&mb, 5 + i, &ib));
    CHECK(ia.width == 160);
    CHECK(ia.height == 120);
    CHECK(ib.width == 320);
    CHECK(ib.height == 240);
  }

  /// 同時読み込み結果が単体読み込みと一致
  REQUIRE(read_frame(&ma, 3, &ia));
  CHECK(same_image(&ia, &ra3));
  REQUIRE(read_frame(&mb, 5, &ib));
  CHECK(same_image(&ib, &rb5));
}
