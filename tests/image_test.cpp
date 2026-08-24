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
