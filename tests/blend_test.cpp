#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>

using namespace mu;

namespace {
Ref<Image> make_flat(int w, int h, Vec4b color) {
  auto img = cutil::make_ref<Image>();
  img->resize(w, h);
  img->has_alpha = true;
  img->fill_rgba(color);
  return img;
}
} // namespace

TEST_CASE("Image::copyto: Blend_Alpha(既定)は従来通りの単純アルファブレンド") {
  auto dst = make_flat(10, 10, Vec4b(100, 100, 100, 255));
  auto src = make_flat(10, 10, Vec4b(200, 200, 200, 255));
  src->copyto(dst.get(), Vec2d(0, 0), 1.0f, Blend_Alpha);
  CHECK(dst->rgba(5, 5) == Vec4b(200, 200, 200, 255));
}

TEST_CASE("Image::copyto: Blend_Addは加算合成される(#31)") {
  auto dst = make_flat(10, 10, Vec4b(100, 50, 0, 255));
  auto src = make_flat(10, 10, Vec4b(50, 50, 50, 255));
  src->copyto(dst.get(), Vec2d(0, 0), 1.0f, Blend_Add);
  auto px = dst->rgba(5, 5);
  CHECK(px[0] == 150); // min(100+50,255)
  CHECK(px[1] == 100); // min(50+50,255)
  CHECK(px[2] == 50);  // min(0+50,255)
}

TEST_CASE("Image::copyto: Blend_Mulは乗算合成される(#31)") {
  auto dst = make_flat(10, 10, Vec4b(200, 200, 200, 255));
  auto src = make_flat(10, 10, Vec4b(128, 128, 128, 255));
  src->copyto(dst.get(), Vec2d(0, 0), 1.0f, Blend_Mul);
  auto px = dst->rgba(5, 5);
  CHECK(px[0] == 200 * 128 / 255);
}

TEST_CASE("Image::copyto: Blend_Screenはスクリーン合成される(#31)") {
  auto dst = make_flat(10, 10, Vec4b(100, 100, 100, 255));
  auto src = make_flat(10, 10, Vec4b(50, 50, 50, 255));
  src->copyto(dst.get(), Vec2d(0, 0), 1.0f, Blend_Screen);
  auto px      = dst->rgba(5, 5);
  int expected = 255 - (255 - 100) * (255 - 50) / 255;
  CHECK(px[0] == expected);
}

TEST_CASE("Image::copyto: Blend_Darken/Lightenはmin/maxで合成される(#31)") {
  auto dst_d = make_flat(10, 10, Vec4b(200, 200, 200, 255));
  auto src_d = make_flat(10, 10, Vec4b(50, 50, 50, 255));
  src_d->copyto(dst_d.get(), Vec2d(0, 0), 1.0f, Blend_Darken);
  CHECK(dst_d->rgba(5, 5)[0] == 50);

  auto dst_l = make_flat(10, 10, Vec4b(200, 200, 200, 255));
  auto src_l = make_flat(10, 10, Vec4b(50, 50, 50, 255));
  src_l->copyto(dst_l.get(), Vec2d(0, 0), 1.0f, Blend_Lighten);
  CHECK(dst_l->rgba(5, 5)[0] == 200);
}

TEST_CASE("Image::copyto: alpha_mulが1未満ならブレンドモードでも中間値になる(#31)") {
  auto dst = make_flat(10, 10, Vec4b(0, 0, 0, 255));
  auto src = make_flat(10, 10, Vec4b(255, 255, 255, 255)); // Blend_Screen(0,255)=255
  src->copyto(dst.get(), Vec2d(0, 0), 0.5f, Blend_Screen);
  auto px = dst->rgba(5, 5);
  CHECK(px[0] > 0);
  CHECK(px[0] < 255);
}
