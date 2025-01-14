#pragma once
#include <string>
#include <vector>

namespace mu::detail {

std::vector<std::string> get_backtrace();
void enable_signal_handlers();
void _mu_assert_fail(const char* file, int line, const char* msg1);

} // namespace mu::detail

// clang-format off
#ifndef MU_ASSERT
#define MU_ASSERT(A) if(!(A)) mu::detail::_mu_assert_fail(__FILE__, __LINE__, #A)
#endif

#ifndef DISP
#define DISP(A) std::cout << #A << " = " << (A) << std::endl
#endif

#ifndef MU_FAIL
#define MU_FAIL(A) mu::detail::_mu_assert_fail(__FILE__, __LINE__, A)
#endif

#ifndef MU_UNUSED
#define MU_UNUSED(A) (void)(A)
#endif
// clang-format on
