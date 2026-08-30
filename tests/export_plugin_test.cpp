#include <cutil/prop.hpp>
#include <doctest/doctest.h>
#include <filesystem>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/plugin.hpp>
#include <movutl/render2d/renderer.hpp>
#include <opencv2/opencv.hpp>

#include "scene_helpers.hpp"

using namespace mu;
namespace fs = std::filesystem;

namespace {
OutputPluginTable* find_output_plugin(const char* name) {
  for(auto& p : detail::AppMain::Get()->output_plugins)
    if(std::string(p.name) == name) return &p;
  return nullptr;
}

// フレーム内でbgr(許容誤差COLOR_TOL)に最も近い最大の連結領域を返す(見つからなければ空)
constexpr int kColorTol = 60;
std::vector<cv::Point> largest_contour_matching_color(const cv::Mat& frame, const cv::Scalar& bgr) {
  cv::Mat mask;
  cv::inRange(frame, bgr - cv::Scalar(kColorTol, kColorTol, kColorTol), bgr + cv::Scalar(kColorTol, kColorTol, kColorTol), mask);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if(contours.empty()) return {};
  return *std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
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

TEST_CASE("Output Plugin: 図形/テキスト/動画を合成した映像を書き出しOpenCV(findContours)で描画結果を検証する") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("Video (FFmpeg)");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  constexpr int kNumFrames = 10;
  auto scene               = mu::test::make_visual_test_scene("../assets/movies/big_buck_bunny_360_10s.mp4", kNumFrames);
  auto& comp               = *scene.comp;

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_video_scene_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);
  const fs::path out_file = out_dir / "scene.mp4";

  void* handle = plg->fn_open(out_file.string().c_str(), comp.size[0], comp.size[1], comp.framerate, plg->defaults);
  REQUIRE(handle != nullptr);

  CPURenderer renderer;
  Ref<Image> frame_buf;
  for(int f = 0; f < kNumFrames; f++) {
    REQUIRE(renderer.render_frame(&comp, f, frame_buf));
    CHECK(plg->fn_write_frame(handle, frame_buf.get(), f));
  }
  CHECK(plg->fn_close(handle));
  REQUIRE(fs::exists(out_file));

  // 書き出した動画をOpenCVで開き、実際に図形/テキスト/映像が合成されていることを確認する
  cv::VideoCapture cap(out_file.string());
  REQUIRE(cap.isOpened());
  std::vector<cv::Mat> frames;
  cv::Mat f;
  while(cap.read(f)) frames.push_back(f.clone());
  cap.release();
  REQUIRE(frames.size() >= (size_t)kNumFrames - 2); // エンコーダの端数でわずかに前後する場合がある

  const cv::Mat& frame = frames[frames.size() / 2]; // ウォームアップの影響を避け中間フレームを見る
  CHECK(frame.cols == comp.size[0]);
  CHECK(frame.rows == comp.size[1]);

  // 赤い矩形が塗りつぶし矩形として検出できること(extent=面積/bbox面積が高いほど矩形らしい。円は約0.78)
  auto rect_contour = largest_contour_matching_color(frame, cv::Scalar(0, 0, 255));
  REQUIRE_MESSAGE(!rect_contour.empty(), "red rectangle not found in exported video");
  cv::Rect rect_bbox = cv::boundingRect(rect_contour);
  double rect_extent = cv::contourArea(rect_contour) / (double)(rect_bbox.width * rect_bbox.height);
  CHECK(rect_extent > 0.75);
  CHECK(std::abs(rect_bbox.x - (int)scene.rect->pos_[0]) < 15);
  CHECK(std::abs(rect_bbox.y - (int)scene.rect->pos_[1]) < 15);

  // 緑の円が円形として検出できること(円形度4πA/P^2が1に近いほど円らしい)
  auto circ_contour = largest_contour_matching_color(frame, cv::Scalar(0, 255, 0));
  REQUIRE_MESSAGE(!circ_contour.empty(), "green circle not found in exported video");
  double circ_area      = cv::contourArea(circ_contour);
  double circ_perimeter = cv::arcLength(circ_contour, true);
  double circularity    = circ_perimeter > 0 ? 4 * CV_PI * circ_area / (circ_perimeter * circ_perimeter) : 0;
  CHECK(circularity > 0.6);

  // テキスト領域に背景と異なるエッジ(文字の輪郭)が存在すること
  cv::Rect text_roi((int)scene.text->pos_[0], (int)scene.text->pos_[1], (int)(comp.size[0] * 0.4), (int)(comp.size[1] * 0.2));
  cv::Mat text_gray, text_edges;
  cv::cvtColor(frame(text_roi), text_gray, cv::COLOR_BGR2GRAY);
  cv::Canny(text_gray, text_edges, 50, 150);
  CHECK(cv::countNonZero(text_edges) > 5);

  // 図形と重ならない領域(右上の隅)で背景動画がベタ塗りでなく実際に合成されていること
  cv::Rect bg_roi(comp.size[0] - 20, 0, 20, 20);
  cv::Scalar mean, stddev;
  cv::meanStdDev(frame(bg_roi), mean, stddev);
  CHECK(stddev[0] > 5.0);

  fs::remove_all(out_dir);
}
