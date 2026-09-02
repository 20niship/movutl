#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/default/image_advcolor_filter.hpp>
#include <movutl/plugin/default/image_distortion_filter.hpp>
#include <movutl/plugin/default/image_gradient_filter.hpp>
#include <movutl/plugin/default/image_posterize_filter.hpp>
#include <movutl/plugin/default/image_vintage_filter.hpp>
#include <movutl/plugin/filter.hpp>

using namespace mu;

namespace {
FilterInData make_fin(Image* img) {
  FilterInData fin;
  fin.img = img;
  return fin;
}
} // namespace

TEST_CASE("4色グラデーション: 四隅がそれぞれの指定色に近づく") {
  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color_tl", Vec4b(255, 0, 0, 255));
  p.set<Vec4b>("color_tr", Vec4b(0, 255, 0, 255));
  p.set<Vec4b>("color_bl", Vec4b(0, 0, 255, 255));
  p.set<Vec4b>("color_br", Vec4b(255, 255, 0, 255));
  p.set<float>("opacity", 100.0f);
  CHECK(mu::detail::f_four_color_gradient.fn_proc(nullptr, &fin, p));
  CHECK(img(0, 0)[0] > img(9, 0)[0]);   // 左上は赤寄り
  CHECK(img(9, 0)[1] > img(0, 0)[1]);   // 右上は緑寄り
}

TEST_CASE("放射グラデーション: 中心が中心色、外側が外側色に近づく") {
  Image img(20, 20);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color1", Vec4b(255, 255, 255, 255));
  p.set<Vec4b>("color2", Vec4b(0, 0, 0, 255));
  p.set<float>("radius", 50.0f);
  p.set<float>("center_x", 50.0f);
  p.set<float>("center_y", 50.0f);
  p.set<float>("opacity", 100.0f);
  CHECK(mu::detail::f_radial_gradient.fn_proc(nullptr, &fin, p));
  CHECK(img(10, 10)[0] > img(0, 0)[0]); // 中心の方が外側より明るい
}

TEST_CASE("斜めクリッピング: 境界より片側が透明になる") {
  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(255, 255, 255, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("angle", 0.0f); // angle=0の法線はx方向を向く(左右で分かれる境界)
  p.set<float>("offset", 0.0f);
  p.set<float>("edge", 1.0f);
  p.set<bool>("invert", false);
  CHECK(mu::detail::f_diagonal_clip.fn_proc(nullptr, &fin, p));
  CHECK(img(0, 5)[3] > img(9, 5)[3]); // 左側が残り右側が消える
}

TEST_CASE("円形クリッピング: 円の外側が透明になる") {
  Image img(20, 20);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(255, 255, 255, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("radius", 20.0f);
  p.set<float>("center_x", 50.0f);
  p.set<float>("center_y", 50.0f);
  p.set<float>("edge", 1.0f);
  p.set<bool>("invert", false);
  CHECK(mu::detail::f_circle_clip.fn_proc(nullptr, &fin, p));
  CHECK(img(10, 10)[3] > img(0, 0)[3]); // 中心が残り四隅が消える
}

TEST_CASE("ビネット: 中心より周辺が暗くなる") {
  Image img(20, 20);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 100.0f);
  p.set<float>("radius", 0.0f);
  CHECK(mu::detail::f_vignette.fn_proc(nullptr, &fin, p));
  CHECK(img(10, 10)[0] > img(0, 0)[0]);
}

TEST_CASE("走査線: 一部の行が暗くなる") {
  Image img(4, 4);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 100.0f);
  p.set<float>("line_width", 1.0f);
  CHECK(mu::detail::f_scanline.fn_proc(nullptr, &fin, p));
  CHECK(img(0, 0)[0] != img(0, 1)[0]); // 隣接行で明るさが交互に変わる
}

TEST_CASE("ポスタリゼーション: levels=2で0か255のどちらかになる") {
  Image img(1, 3);
  img[0] = Vec4b(10, 10, 10, 255);
  img[1] = Vec4b(130, 130, 130, 255);
  img[2] = Vec4b(250, 250, 250, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("levels", 2.0f);
  CHECK(mu::detail::f_posterize.fn_proc(nullptr, &fin, p));
  for(size_t i = 0; i < img.size(); i++) CHECK((img[i][0] == 0 || img[i][0] == 255));
}

TEST_CASE("2値化: しきい値未満は0、以上は255になる") {
  Image img(1, 2);
  img[0] = Vec4b(50, 50, 50, 255);
  img[1] = Vec4b(200, 200, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("threshold", 128.0f);
  CHECK(mu::detail::f_binarize.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 0);
  CHECK(img[1][0] == 255);
}

TEST_CASE("レンズ歪み: strength=0なら画素は変わらない") {
  Image img(4, 4);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(50, 60, 70, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 0.0f);
  CHECK(mu::detail::f_lens_distortion.fn_proc(nullptr, &fin, p));
  CHECK(img(1, 1)[0] == 50);
}

TEST_CASE("揺らぎ: amplitude=0なら画素は変わらない") {
  Image img(4, 4);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(50, 60, 70, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("amplitude", 0.0f);
  p.set<float>("frequency", 0.1f);
  CHECK(mu::detail::f_wave_distortion.fn_proc(nullptr, &fin, p));
  CHECK(img(1, 1)[0] == 50);
}

TEST_CASE("万華鏡: segments指定で画素がサンプリングし直される(実行できること)") {
  Image img(20, 20);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  img(15, 10) = Vec4b(255, 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("segments", 4.0f);
  CHECK(mu::detail::f_kaleidoscope.fn_proc(nullptr, &fin, p));
}

TEST_CASE("カラーバランス: ハイライトのオフセットが明部に加算される") {
  Image img(1, 1);
  img[0] = Vec4b(240, 240, 240, 255); // 明るい画素
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("highlight_r", 10.0f);
  p.set<float>("shadow_r", -10.0f);
  CHECK(mu::detail::f_color_balance.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] > 240); // 明部なのでhighlightの加算が支配的
}

TEST_CASE("カラーLUT: gamma=100(1.0)なら画素は変わらない") {
  Image img(1, 1);
  img[0] = Vec4b(100, 150, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("gamma_r", 100.0f);
  p.set<float>("gamma_g", 100.0f);
  p.set<float>("gamma_b", 100.0f);
  CHECK(mu::detail::f_color_lut.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 100);
  CHECK(img[0][1] == 150);
  CHECK(img[0][2] == 200);
}

TEST_CASE("ソフトフォーカス: opacity=0なら画素は変わらない") {
  Image img(3, 3);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(100, 100, 100, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 5.0f);
  p.set<float>("opacity", 0.0f);
  CHECK(mu::detail::f_soft_focus.fn_proc(nullptr, &fin, p));
  CHECK(img(1, 1)[0] == 100);
}

TEST_CASE("インターレースシフト: 偶数行は変わらず奇数行だけシフトする") {
  Image img(5, 3);
  for(int y = 0; y < 3; y++)
    for(int x = 0; x < 5; x++) img(x, y) = Vec4b((uint8_t)(x * 10), 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("shift", 1.0f);
  CHECK(mu::detail::f_interlace_shift.fn_proc(nullptr, &fin, p));
  CHECK(img(2, 0)[0] == 20);  // 偶数行(y=0)は変化しない
  CHECK(img(2, 1)[0] == 10);  // 奇数行(y=1)は1pxシフトしている
}
