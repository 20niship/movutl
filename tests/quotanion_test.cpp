#include <doctest/doctest.h>
#include <movutl/core/quotanion.hpp>

using namespace mu;
TEST_CASE("quotanion") {
  Quat q;
  CHECK(q.w == 1);
}
