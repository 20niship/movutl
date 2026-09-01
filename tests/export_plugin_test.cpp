#include <cutil/prop.hpp>
#include <doctest/doctest.h>
#include <filesystem>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/audio/audio_mixer.hpp>
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

InputPluginTable* find_input_plugin(const char* name) {
  for(auto& p : detail::AppMain::Get()->input_plugins)
    if(std::string(p.name) == name) return &p;
  return nullptr;
}

// フレーム内でbgr(許容誤差kColorTol)に最も近い最大の連結領域を返す(見つからなければ空)
constexpr int kColorTol = 60;
std::vector<cv::Point> largest_contour_matching_color(const cv::Mat& frame, const cv::Scalar& bgr) {
  cv::Mat mask;
  cv::inRange(frame, bgr - cv::Scalar(kColorTol, kColorTol, kColorTol), bgr + cv::Scalar(kColorTol, kColorTol, kColorTol), mask);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if(contours.empty()) return {};
  return *std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
}

// make_visual_test_sceneが構築した赤矩形/緑円/白文字/左右2本の背景動画が1フレームに正しく合成されていることを検証する
void verify_scene_frame(const cv::Mat& frame, const mu::test::VisualTestScene& scene) {
  CHECK(frame.cols == mu::test::kSceneWidth);
  CHECK(frame.rows == mu::test::kSceneHeight);

  // 赤い矩形が塗りつぶし矩形として検出できること(extent=面積/bbox面積が高いほど矩形らしい。円は約0.78)
  auto rect_contour = largest_contour_matching_color(frame, cv::Scalar(0, 0, 255));
  REQUIRE_MESSAGE(!rect_contour.empty(), "red rectangle not found");
  cv::Rect rect_bbox = cv::boundingRect(rect_contour);
  double rect_extent = cv::contourArea(rect_contour) / (double)(rect_bbox.width * rect_bbox.height);
  CHECK(rect_extent > 0.75);
  CHECK(std::abs(rect_bbox.x - (int)scene.rect->pos_[0]) < 20);
  CHECK(std::abs(rect_bbox.y - (int)scene.rect->pos_[1]) < 20);

  // 緑の円が円形として検出できること(円形度4πA/P^2が1に近いほど円らしい)
  auto circ_contour = largest_contour_matching_color(frame, cv::Scalar(0, 255, 0));
  REQUIRE_MESSAGE(!circ_contour.empty(), "green circle not found");
  double circ_area      = cv::contourArea(circ_contour);
  double circ_perimeter = cv::arcLength(circ_contour, true);
  double circularity    = circ_perimeter > 0 ? 4 * CV_PI * circ_area / (circ_perimeter * circ_perimeter) : 0;
  CHECK(circularity > 0.6);

  // テキスト領域に背景と異なるエッジ(文字の輪郭)が存在すること
  cv::Rect text_roi((int)scene.text->pos_[0], (int)scene.text->pos_[1], (int)(mu::test::kSceneWidth * 0.4), (int)(mu::test::kSceneHeight * 0.2));
  cv::Mat text_gray, text_edges;
  cv::cvtColor(frame(text_roi), text_gray, cv::COLOR_BGR2GRAY);
  cv::Canny(text_gray, text_edges, 50, 150);
  CHECK(cv::countNonZero(text_edges) > 5);

  // 図形と重ならない右上の隅で、背景動画がベタ塗りでなく実際に合成されていること
  cv::Scalar mean, stddev;
  cv::meanStdDev(frame(cv::Rect(mu::test::kSceneWidth - 20, 0, 20, 20)), mean, stddev);
  CHECK(stddev[0] > 5.0);
}

} // namespace

TEST_CASE("Output Plugin: fn_open/fn_write_frame/fn_closeを直接呼び出してpngファイルが生成される") {
  detail::init_external_plugins();
  auto* plg = find_output_plugin("PNG Image/Sequence");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_init(&plg->props, &plg->defaults));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_png_direct_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);

  void* handle = plg->fn_open((out_dir / "frame.png").string().c_str(), 4, 4, 30.0f, 0, 0, plg->defaults);
  REQUIRE(handle != nullptr);
  Image img(4, 4);
  img.fill_rgba(Vec4b(10, 20, 30, 255));
  CHECK(plg->fn_write_frame(handle, &img, 0));
  CHECK(plg->fn_close(handle));
  CHECK(fs::exists(out_dir / "frame_000000.png"));

  fs::remove_all(out_dir);
}

TEST_CASE("Output Plugin: 登録済みの全プラグイン×全対応拡張子でCompositionをレンダリング・エクスポートでき、合成結果が正しいことを確認する") {
  detail::init_external_plugins();
  auto& plugins = detail::AppMain::Get()->output_plugins;
  REQUIRE(plugins.size() > 0);

  constexpr int kNumFrames = 5;
  auto scene               = mu::test::make_visual_test_scene(kNumFrames);
  auto& comp               = *scene.comp;

  // 全プラグイン/拡張子で使い回せるよう、レンダリングは1回だけ行う
  CPURenderer renderer;
  std::vector<Ref<Image>> frames(kNumFrames);
  for(int f = 0; f < kNumFrames; f++) REQUIRE(renderer.render_frame(&comp, f, frames[f]));

  const fs::path out_dir = fs::temp_directory_path() / "movutl_export_all_plugins_test";
  fs::remove_all(out_dir);
  fs::create_directories(out_dir);

  int plugin_idx = 0;
  for(auto& plg : plugins) {
    REQUIRE_MESSAGE(plg.fn_init(&plg.props, &plg.defaults), "fn_init failed for " << plg.name);

    for(auto* ext : plg.extensions) {
      if(ext == nullptr || ext[0] == '\0') continue;
      INFO("plugin=" << plg.name << " ext=" << ext);

      const std::string stem  = "scene_" + std::to_string(plugin_idx);
      const fs::path out_file = out_dir / (stem + "." + ext);
      void* handle            = plg.fn_open(out_file.string().c_str(), comp.size[0], comp.size[1], comp.framerate, comp.audio_sample_rate, comp.audio_channels, plg.defaults);
      REQUIRE(handle != nullptr);
      for(int f = 0; f < kNumFrames; f++) {
        CHECK(plg.fn_write_frame(handle, frames[f].get(), f));
        if(plg.fn_write_audio != nullptr) {
          int64_t s0 = comp.frame_to_sample(f);
          int64_t s1 = comp.frame_to_sample(f + 1);
          int n      = (int)std::max<int64_t>(1, s1 - s0);
          std::vector<int16_t> audio_buf((size_t)n * comp.audio_channels);
          mix_audio_range(&comp, s0, n, audio_buf.data());
          CHECK(plg.fn_write_audio(handle, audio_buf.data(), n));
        }
      }
      CHECK(plg.fn_close(handle));

      if(std::string(plg.name) == "Audio (FFmpeg)") {
        // 音声専用プラグイン(映像フレームは持たない)。映像デコード検証はスキップし、音声が入っていることのみ確認する
        REQUIRE(fs::exists(out_file));
        CHECK(fs::file_size(out_file) > 0);

        auto* reader = find_input_plugin("FFmpeg Video Reader");
        REQUIRE(reader != nullptr);
        InputHandle rh = reader->fn_open(out_file.string().c_str());
        REQUIRE(rh != nullptr);
        EntityInfo info;
        REQUIRE(reader->fn_info_get(rh, &info));
        CHECK_MESSAGE(info.audio_n > 0, "exported " << ext << " has no audio track");
        reader->fn_close(rh);
      } else if(plg.is_sequence) {
        const fs::path first_frame = out_dir / (stem + "_000000." + ext);
        REQUIRE(fs::exists(first_frame));
        auto loaded = Image::Create("loaded", first_frame.string().c_str());
        REQUIRE(loaded->width > 0);
        cv::Mat frame(loaded->height, loaded->width, CV_8UC4, loaded->data());
        cv::Mat frame_bgr;
        cv::cvtColor(frame, frame_bgr, cv::COLOR_BGRA2BGR);
        verify_scene_frame(frame_bgr, scene);
      } else {
        REQUIRE(fs::exists(out_file));
        CHECK(fs::file_size(out_file) > 0);

        cv::VideoCapture cap(out_file.string());
        REQUIRE(cap.isOpened());
        std::vector<cv::Mat> decoded;
        cv::Mat f;
        while(cap.read(f)) decoded.push_back(f.clone());
        cap.release();
        REQUIRE(decoded.size() >= (size_t)kNumFrames - 2); // エンコーダの端数でわずかに前後する場合がある
        verify_scene_frame(decoded[decoded.size() / 2], scene);

        if(plg.fn_write_audio != nullptr) {
          auto* reader = find_input_plugin("FFmpeg Video Reader");
          REQUIRE(reader != nullptr);
          InputHandle rh = reader->fn_open(out_file.string().c_str());
          REQUIRE(rh != nullptr);
          EntityInfo info;
          REQUIRE(reader->fn_info_get(rh, &info));
          CHECK_MESSAGE(info.audio_n > 0, "exported " << ext << " has no audio track");
          reader->fn_close(rh);
        }
      }
    }
    plugin_idx++;
  }

  fs::remove_all(out_dir);
}
