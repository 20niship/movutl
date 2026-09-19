#include <doctest/doctest.h>
#include <movutl/asset/image.hpp>

using namespace mu;

namespace {
Image make_src(int w, int h) {
  Image s(w, h);
  s.has_alpha = true;
  for(size_t i = 0; i < s.size(); i++) s[i] = Vec4b(255, 0, 0, 255);
  return s;
}
Image make_target(int w, int h) {
  Image t(w, h);
  t.has_alpha = true;
  for(size_t i = 0; i < t.size(); i++) t[i] = Vec4b(0, 0, 0, 0);
  return t;
}
// 中央行で不透明な画素の数
int opaque_in_row(const Image& img, int y) {
  int n = 0;
  for(size_t x = 0; x < img.width; x++)
    if(img(x, y)[3] > 0) n++;
  return n;
}
} // namespace

TEST_CASE("Entity::composite: 拡大率のX/Yを別々に反映する") {
  auto src    = make_src(4, 4);
  auto ent    = cutil::make_ref<Image>();
  ent->scale_ = Vec2(200, 100); // 幅2倍・高さ等倍
  auto t      = make_target(20, 20);
  REQUIRE(ent->composite(src, &t));
  CHECK(opaque_in_row(t, 10) == 8);
  CHECK(opaque_in_row(t, 10 - 3) == 0); // 高さは4pxのまま
}

TEST_CASE("Entity::composite: 縦横比aspectが正だと横が縮み、負だと縦が縮む") {
  auto src     = make_src(8, 8);
  auto ent     = cutil::make_ref<Image>();
  ent->aspect_ = 0.5f;
  auto t       = make_target(20, 20);
  REQUIRE(ent->composite(src, &t));
  CHECK(opaque_in_row(t, 10) == 4);

  ent->aspect_ = -0.5f;
  auto u       = make_target(20, 20);
  REQUIRE(ent->composite(src, &u));
  CHECK(opaque_in_row(u, 10) == 8);
  int col = 0;
  for(size_t y = 0; y < u.height; y++) col += u(10, y)[3] > 0;
  CHECK(col == 4);
}

TEST_CASE("Entity::composite: Y軸回転で見かけの幅が狭まる") {
  auto src = make_src(20, 20);
  auto ent = cutil::make_ref<Image>();
  auto a   = make_target(40, 40);
  REQUIRE(ent->composite(src, &a));
  ent->rot_y_ = 60.f;
  auto b      = make_target(40, 40);
  REQUIRE(ent->composite(src, &b));
  CHECK(opaque_in_row(b, 20) > 0);
  CHECK(opaque_in_row(b, 20) < opaque_in_row(a, 20));
}

TEST_CASE("Entity::composite: 位置は整数に丸められず小数の回転中心でも欠けない") {
  auto src       = make_src(10, 10);
  auto ent       = cutil::make_ref<Image>();
  ent->rotation_ = 30.f;
  ent->pos_      = Vec3(0.5f, 0.5f, 0);
  auto t         = make_target(40, 40);
  REQUIRE(ent->composite(src, &t));
  CHECK(t(20, 20)[3] == 255);
}
