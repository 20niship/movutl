#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/default/image_effect_filter.hpp>
#include <movutl/plugin/default/image_glow_filter.hpp>
#include <movutl/plugin/default/image_key_filter.hpp>
#include <movutl/plugin/default/image_outline_filter.hpp>
#include <movutl/plugin/default/image_tone_filter.hpp>
#include <movutl/plugin/filter.hpp>

using namespace mu;

namespace {
FilterInData make_fin(Image* img) {
  FilterInData fin;
  fin.img = img;
  return fin;
}
} // namespace

TEST_CASE("クロマキー: しきい値内の色は透明になる") {
  Image img(1, 1);
  img[0] = Vec4b(0, 255, 0, 255); // 緑
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("key_color", Vec4b(0, 255, 0, 255));
  p.set<float>("threshold", 30.0f);
  p.set<float>("edge", 10.0f);
  CHECK(mu::detail::f_chroma_key.fn_proc(nullptr, &fin, p));
  CHECK(img[0][3] == 0);
}

TEST_CASE("クロマキー: しきい値外の色はそのまま") {
  Image img(1, 1);
  img[0] = Vec4b(255, 0, 0, 255); // 赤(キー色=緑から遠い)
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("key_color", Vec4b(0, 255, 0, 255));
  p.set<float>("threshold", 30.0f);
  p.set<float>("edge", 10.0f);
  CHECK(mu::detail::f_chroma_key.fn_proc(nullptr, &fin, p));
  CHECK(img[0][3] == 255);
}

TEST_CASE("ルミナンスキー: 暗い部分がデフォルトで抜ける") {
  Image img(1, 1);
  img[0] = Vec4b(0, 0, 0, 255); // 黒(輝度0)
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("threshold", 50.0f);
  p.set<float>("edge", 10.0f);
  p.set<bool>("invert", false);
  CHECK(mu::detail::f_luminance_key.fn_proc(nullptr, &fin, p));
  CHECK(img[0][3] == 0);
}

TEST_CASE("ルミナンスキー: invert=trueで明るい部分が抜ける") {
  Image img(1, 1);
  img[0] = Vec4b(255, 255, 255, 255); // 白(輝度255)
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("threshold", 50.0f);
  p.set<float>("edge", 10.0f);
  p.set<bool>("invert", true);
  CHECK(mu::detail::f_luminance_key.fn_proc(nullptr, &fin, p));
  CHECK(img[0][3] == 0);
}

TEST_CASE("発光: しきい値以上の明部が周囲を明るくする") {
  Image img(5, 1);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  img[2] = Vec4b(255, 255, 255, 255); // 中央だけ明るい
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("threshold", 200.0f);
  p.set<float>("range", 3.0f);
  p.set<float>("intensity", 200.0f);
  CHECK(mu::detail::f_glow.fn_proc(nullptr, &fin, p));
  CHECK(img[1][0] > 0); // 隣接ピクセルに光が滲んでいる
}

TEST_CASE("グロー: 全体をぼかして明るさが混ざる") {
  Image img(5, 1);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  img[2] = Vec4b(255, 255, 255, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("range", 3.0f);
  p.set<float>("intensity", 100.0f);
  CHECK(mu::detail::f_bloom.fn_proc(nullptr, &fin, p));
  CHECK(img[1][0] > 0);
}

TEST_CASE("縁取り: 透明部分に指定色の縁が付く") {
  Image img(5, 5);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 0);
  img(2, 2) = Vec4b(255, 255, 255, 255); // 中央だけ不透明
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<Vec4b>("color", Vec4b(255, 0, 0, 255));
  p.set<float>("width", 1.0f);
  CHECK(mu::detail::f_outline.fn_proc(nullptr, &fin, p));
  CHECK(img(1, 2)[3] > 0); // 隣接ピクセルに縁ができている
  CHECK(img(1, 2)[0] == 255);
}

TEST_CASE("クリッピング&リサイズ: クロップ範囲外の色が消え元サイズへ戻る") {
  Image img(4, 1);
  img[0] = Vec4b(255, 0, 0, 255);
  img[1] = Vec4b(0, 255, 0, 255);
  img[2] = Vec4b(0, 0, 255, 255);
  img[3] = Vec4b(255, 255, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("left", 1.0f);
  p.set<float>("top", 0.0f);
  p.set<float>("right", 1.0f);
  p.set<float>("bottom", 0.0f);
  p.set<bool>("resize", true);
  CHECK(mu::detail::f_clipping.fn_proc(nullptr, &fin, p));
  CHECK(img.width == 4); // resize=trueで元サイズへ戻る
}

TEST_CASE("反転: RGBが255から引いた値になる") {
  Image img(1, 1);
  img[0] = Vec4b(10, 100, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<bool>("invert_alpha", false);
  CHECK(mu::detail::f_invert.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 245);
  CHECK(img[0][1] == 155);
  CHECK(img[0][2] == 55);
  CHECK(img[0][3] == 255); // alphaは反転しない設定
}

TEST_CASE("モノクロ: RGB成分が全て同じ値になる") {
  Image img(1, 1);
  img[0] = Vec4b(10, 100, 200, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 100.0f);
  CHECK(mu::detail::f_grayscale.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == img[0][1]);
  CHECK(img[0][1] == img[0][2]);
}

TEST_CASE("セピア: 元がグレーではRGBの比率が偏る") {
  Image img(1, 1);
  img[0] = Vec4b(128, 128, 128, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 100.0f);
  CHECK(mu::detail::f_sepia.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] > img[0][1]); // R > G > B のセピア調
  CHECK(img[0][1] > img[0][2]);
}

TEST_CASE("シャープ: strength=0なら画素は変わらない") {
  Image img(3, 3);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(100, 100, 100, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("strength", 0.0f);
  CHECK(mu::detail::f_sharpen.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 100);
}

TEST_CASE("エッジ抽出: 単色画像はエッジ無しで真っ黒になる") {
  Image img(9, 9);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(128, 128, 128, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("threshold", 100.0f);
  CHECK(mu::detail::f_edge_detect.fn_proc(nullptr, &fin, p));
  CHECK(img[0][0] == 0);
}

TEST_CASE("モザイク: ブロック内が均一な値になる") {
  Image img(4, 4);
  img(0, 0) = Vec4b(255, 0, 0, 255);
  img(1, 0) = Vec4b(0, 0, 0, 255);
  img(0, 1) = Vec4b(0, 0, 0, 255);
  img(1, 1) = Vec4b(0, 0, 0, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("block_size", 2.0f);
  CHECK(mu::detail::f_mosaic.fn_proc(nullptr, &fin, p));
  CHECK(img(0, 0)[0] == img(1, 1)[0]); // 同一ブロック内は均一化される
}

TEST_CASE("リサイズ: scale=200で画像サイズが2倍になる") {
  Image img(4, 4);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(100, 100, 100, 255);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("scale", 200.0f);
  CHECK(mu::detail::f_resize.fn_proc(nullptr, &fin, p));
  CHECK(img.width == 8);
  CHECK(img.height == 8);
}

TEST_CASE("リサイズ: scale=100なら何もしない") {
  Image img(4, 4);
  auto fin = make_fin(&img);
  cutil::Prop p;
  p.set<float>("scale", 100.0f);
  CHECK(mu::detail::f_resize.fn_proc(nullptr, &fin, p));
  CHECK(img.width == 4);
}
