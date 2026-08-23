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
