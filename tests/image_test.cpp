#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;

TEST_CASE("STB Image Reader: PNG読み込み") {
  detail::init_external_plugins();
  auto img = Image::Create("test_png", "../assets/textures/concrete.png");
  CHECK(img->width > 0);
  CHECK(img->height > 0);
  bool nonzero = false;
  for(size_t i = 0; i < img->size(); i++)
    if(img->data()[i][0] != 0 || img->data()[i][1] != 0 || img->data()[i][2] != 0) nonzero = true;
  CHECK(nonzero);
}

TEST_CASE("STB Image Reader: JPG読み込み") {
  detail::init_external_plugins();
  auto img = Image::Create("test_jpg", "../assets/textures/tile.jpg");
  CHECK(img->width > 0);
  CHECK(img->height > 0);
}

TEST_CASE("STB Image Reader: 存在しないファイル") {
  detail::init_external_plugins();
  Image img;
  CHECK_FALSE(img.load_file("/nonexistent/image.png"));
}

TEST_CASE("Image::copyto: 透明ピクセルは合成先を上書きしない") {
  Image dst(4, 4);
  for(size_t i = 0; i < dst.size(); i++) dst[i] = Vec4b(10, 20, 30, 255);

  Image src(2, 2);
  src[0] = Vec4b(255, 0, 0, 0);   // 完全透明: dstを変えないはず
  src[1] = Vec4b(0, 255, 0, 255); // 不透明: dstを上書きするはず
  src[2] = Vec4b(0, 0, 255, 128); // 半透明: dstと混ざるはず
  src[3] = Vec4b(255, 255, 0, 0);

  src.copyto(&dst, Vec2d(0, 0));

  CHECK(dst[0] == Vec4b(10, 20, 30, 255)); // 透明ピクセルは不変
  CHECK(dst[1] == Vec4b(0, 255, 0, 255));  // 不透明ピクセルは上書き
  CHECK(dst[4][0] < 10);                   // 半透明ピクセルは元の色(10,20,30)より青(0,0,255)側へ寄る
  CHECK(dst[4][2] > 30);
}

TEST_CASE("Image::copyto: scale/angle指定時は画像自身の中心を軸にする(dst全体の中心ではない)") {
  Image dst(20, 20);
  for(size_t i = 0; i < dst.size(); i++) dst[i] = Vec4b(0, 0, 0, 255);

  // 2x2の不透明画像をdstの左寄り(pmin=(2,2))に、2倍拡大で配置する
  Image src(2, 2);
  for(size_t i = 0; i < src.size(); i++) src[i] = Vec4b(255, 255, 255, 255);
  src.copyto(&dst, Vec2d(2, 2), 2.0f, 0.0f);

  // 元画像の中心(pmin+1,1)=(3,3)付近が拡大後も白いはず(dst中心(10,10)基準だと(3,3)は白くならない)
  CHECK(dst(3, 3)[0] == 255);
  // dst全体の中心(10,10)は今回の配置(pmin=2,2)からは大きく外れるので白くならないはず
  CHECK(dst(10, 10)[0] == 0);
}

TEST_CASE("Image::drawpoly: 四隅を同一オフセットでずらすと単純平行移動と同じ結果になる") {
  Image dst(10, 10);
  for(size_t i = 0; i < dst.size(); i++) dst[i] = Vec4b(0, 0, 0, 255);

  Image src(4, 4);
  for(size_t i = 0; i < src.size(); i++) src[i] = Vec4b(255, 255, 255, 255);

  Vec2d corners[4] = {Vec2d(2, 2), Vec2d(6, 2), Vec2d(2, 6), Vec2d(6, 6)};
  src.drawpoly(&dst, corners);

  CHECK(dst(3, 3)[0] == 255); // 平行移動後のsrc範囲内
  CHECK(dst(8, 8)[0] == 0);   // src範囲外は元のまま
}

TEST_CASE("Image::drawpoly: 上辺を狭めると台形になり上端中央付近だけ残る") {
  Image dst(20, 20);
  for(size_t i = 0; i < dst.size(); i++) dst[i] = Vec4b(0, 0, 0, 255);

  Image src(10, 10);
  for(size_t i = 0; i < src.size(); i++) src[i] = Vec4b(255, 255, 255, 255);

  // 左上,右上,左下,右下。上辺を(8,10)まで狭め、下辺は(0,20)のまま台形にする
  Vec2d corners[4] = {Vec2d(8, 0), Vec2d(12, 0), Vec2d(0, 20), Vec2d(20, 20)};
  src.drawpoly(&dst, corners);

  CHECK(dst(10, 1)[0] == 255); // 上辺中央付近は狭い上辺内なので残る
  CHECK(dst(1, 1)[0] == 0);    // 上辺左端付近は狭められた範囲外なので黒のまま
}
