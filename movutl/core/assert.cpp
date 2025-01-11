#include <movutl/core/assert.hpp>
#include <movutl/core/logger.hpp>

namespace mu::detail {

void _mu_assert_fail(const char* file, int line, const char* msg1) {
  LOG_F(ERROR, "Assertion failed: %s:%d: %s", file, line, msg1);
  LOG_F(FATAL, "Aborting...");
}

} // namespace mu::detail
