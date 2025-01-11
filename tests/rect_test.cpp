#include <doctest/doctest.h>
#include <iostream>
#include <movutl/core/rect.hpp>

using namespace mu;

TEST_CASE("Rect3D") {
  Rect3D r;
  DISP(r.str());
  r.expand(Vec3{-1, -1, -1});
  DISP(r.str());


  float max = std::numeric_limits<float>::lowest();
  DISP(max);
  DISP(-1 > max);
  max = std::max<float>(-1, max);
  DISP(max);
}
