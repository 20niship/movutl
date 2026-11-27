#include <doctest/doctest.h>
#include <movutl/raster/pixel_buffer.hpp>

using namespace mu;

TEST_CASE("PixelBuffer8/16: resize・fill・アクセス・サイズ") {
  PixelBuffer8 a(4, 3);
  CHECK(a.size() == 12);
  CHECK(a.size_in_bytes() == 12 * 4);
  a.fill(Vec4b{1, 2, 3, 4});
  CHECK(a(3, 2)[2] == 3);

  PixelBuffer16 b(4, 3);
  CHECK(b.size_in_bytes() == 12 * 8);
  b.fill(Vec4w{65535, 0, 0, 65535});
  b(1, 1)[1] = 40000;
  CHECK(b(1, 1)[1] == 40000);
  CHECK(b(0, 0)[0] == 65535);
}
