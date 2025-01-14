#include <movutl/core/quotanion.hpp>
#include <doctest/doctest.h>

using namespace mu;
TEST_CASE("quotanion") {
  Quat q;
  CHECK(q.w == 1);
}
