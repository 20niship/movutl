#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/default/image_blur_filter.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/filter.hpp>

using namespace mu;

namespace {
FilterInData make_fin(Image* img) {
  FilterInData fin;
  fin.img = img;
  return fin;
}
} // namespace

TEST_CASE("色調補正: brightness/contrastが変化なしなら画素は変わらない") {
  Image img(2, 2);
  img[0]   = Vec4b(100, 120, 140, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("brightness", 100.0f);
  p.set<float>("contrast", 100.0f);
  CHECK(mu::detail::f_color_correction.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 100);
  CHECK(img[0][1] == 120);
  CHECK(img[0][2] == 140);
}

TEST_CASE("色調補正: brightness=200で明るくなる") {
  Image img(1, 1);
  img[0]   = Vec4b(100, 100, 100, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("brightness", 200.0f);
  p.set<float>("contrast", 100.0f);
  CHECK(mu::detail::f_color_correction.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] > 100);
}

TEST_CASE("色調補正: hueで色相が回転する(彩度のある色が変化する)") {
  Image img(1, 1);
  img[0]   = Vec4b(255, 0, 0, 255); // 赤
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("hue", 120.0f); // 赤->緑方向へ回転
  p.set<float>("saturation", 100.0f);
  p.set<float>("brightness", 100.0f);
  p.set<float>("contrast", 100.0f);
  CHECK(mu::detail::f_color_correction.fn_proc(nullptr, &fin, p));
  CHECK(img[0][1] > img[0][0]); // 緑成分が支配的になっているはず
}

TEST_CASE("単色化: strength=100で輝度に応じた単色になる") {
  Image img(1, 1);
  img[0]   = Vec4b(200, 50, 10, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color", Vec4b(0, 0, 255, 255)); // 青
  p.set<float>("strength", 100.0f);
  CHECK(mu::detail::f_single_color.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 0); // R成分は消える
  CHECK(img[0][1] == 0); // G成分は消える
  CHECK(img[0][2] > 0);  // B成分だけ輝度分残る
}

TEST_CASE("単色化: strength=0なら画素は変わらない") {
  Image img(1, 1);
  img[0]   = Vec4b(200, 50, 10, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color", Vec4b(0, 0, 255, 255));
  p.set<float>("strength", 0.0f);
  CHECK(mu::detail::f_single_color.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 200);
  CHECK(img[0][1] == 50);
  CHECK(img[0][2] == 10);
}

TEST_CASE("色ずらし: shift_xでRとBが左右逆方向にずれる") {
  Image img(3, 1);
  img[0]   = Vec4b(255, 0, 0, 255); // R
  img[1]   = Vec4b(0, 255, 0, 255); // G
  img[2]   = Vec4b(0, 0, 255, 255); // B
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("shift_x", 1.0f);
  p.set<float>("shift_y", 0.0f);
  CHECK(mu::detail::f_color_shift.fn_proc(nullptr, &fin, p));
  // 中央ピクセル(元はG単色)のRチャンネルは1つ左(元R画素)から、Bチャンネルは1つ右(元B画素)から来る
  CHECK(img[1][0] == 255);
  CHECK(img[1][2] == 255);
}

TEST_CASE("グラデーション: angle=0で左端がcolor1寄り、右端がcolor2寄りになる") {
  Image img(10, 1);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color1", Vec4b(0, 0, 0, 255));
  p.set<Vec4b>("color2", Vec4b(255, 255, 255, 255));
  p.set<float>("angle", 0.0f);
  p.set<float>("opacity", 100.0f);
  CHECK(mu::detail::f_gradient.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] < img[9][0]); // 左端より右端の方が明るい
}

TEST_CASE("拡張色調補正: 各チャンネルへオフセットが加算される") {
  Image img(1, 1);
  img[0]   = Vec4b(100, 100, 100, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("r", 50.0f);
  p.set<float>("g", -30.0f);
  p.set<float>("b", 0.0f);
  CHECK(mu::detail::f_extend_color.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 150);
  CHECK(img[0][1] == 70);
  CHECK(img[0][2] == 100);
}

TEST_CASE("拡張色調補正: オフセットは0-255にクランプされる") {
  Image img(1, 1);
  img[0]   = Vec4b(240, 10, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("r", 50.0f);
  p.set<float>("g", -50.0f);
  p.set<float>("b", 0.0f);
  CHECK(mu::detail::f_extend_color.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 255);
  CHECK(img[0][1] == 0);
}

TEST_CASE("ぼかし: エッジが平滑化される") {
  Image img(5, 1);
  img[0] = img[1] = Vec4b(0, 0, 0, 255);
  img[2] = img[3] = img[4] = Vec4b(255, 255, 255, 255);
  auto fin                 = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 3.0f);
  CHECK(mu::detail::f_blur.fn_proc(nullptr, &fin, p));
  // 境界付近の画素は0/255どちらでもない中間値になっているはず
  CHECK(img[2][0] > 0);
  CHECK(img[2][0] < 255);
}

TEST_CASE("ぼかし: range<=0なら画素は変わらない") {
  Image img(2, 1);
  img[0]   = Vec4b(10, 20, 30, 255);
  img[1]   = Vec4b(200, 210, 220, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 0.0f);
  CHECK(mu::detail::f_blur.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 10);
  CHECK(img[1][0] == 200);
}

TEST_CASE("方向ぼかし: 水平方向にエッジが平滑化される") {
  Image img(5, 1);
  img[0] = img[1] = Vec4b(0, 0, 0, 255);
  img[2] = img[3] = img[4] = Vec4b(255, 255, 255, 255);
  auto fin                 = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 3.0f);
  p.set<float>("angle", 0.0f); // 水平方向
  CHECK(mu::detail::f_directional_blur.fn_proc(nullptr, &fin, p));
  CHECK(img[2][0] > 0);
  CHECK(img[2][0] < 255);
}

TEST_CASE("放射ぼかし: 中心から離れた位置ほど周辺画素と混ざる") {
  Image img(9, 9);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  img[9 * 4 + 8] = Vec4b(255, 255, 255, 255); // 右端中央に白ドット
  auto fin       = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 50.0f);
  CHECK(mu::detail::f_radial_blur.fn_proc(nullptr, &fin, p));
  // ドット自身の値は中心方向へサンプリングされ薄まっているはず(元の255ではなくなる)
  CHECK(img[9 * 4 + 8][0] < 255);
}

TEST_CASE("放射ぼかし: range<=0なら画素は変わらない") {
  Image img(2, 1);
  img[0]   = Vec4b(10, 20, 30, 255);
  img[1]   = Vec4b(200, 210, 220, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 0.0f);
  CHECK(mu::detail::f_radial_blur.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 10);
  CHECK(img[1][0] == 200);
}
