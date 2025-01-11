#pragma once

#include <cassert>
/* #define MU_ASSERT(A) assert((A) "MovUtl assertion failed!  : " #A  __FILE__ __LINE__ ) */
#ifndef MU_ASSERT
#define MU_ASSERT(A) assert(A) 
#endif

#ifndef DISP
#define DISP(A) std::cout << #A << " = " << (A) << std::endl
#endif

void MU_FAIL_IMPL(const char* file, int line, const char* msg);

#ifndef MU_FAIL
#define MU_FAIL(A) MU_FAIL_IMPL(__FILE__, __LINE__, A)
#endif


