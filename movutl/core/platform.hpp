#pragma once
// clang-format off
#ifdef _WIN32
#ifndef NOMINMAX   
#define NOMINMAX
#endif
	#ifdef _WIN64
		#define MOVUTL_PLATFORM_WINDOWS
	#else
		#error "Windows x86 are not supported!"
	#endif
#elif defined(__EMSCRIPTEN__)
		#define MOVUTL_PLATFORM_EMSCRIPTEN
#elif defined(__APPLE__) || defined(__MACH__)
		#define MOVUTL_PLATFORM_MACOS
#elif defined(__ANDROID__)
    #error "Android is not supported!"
#elif defined(__linux__)
	#define MOVUTL_PLATFORM_LINUX
#else
	#error "Unknown platform!"
#endif // End of platform detection

#if __has_include(<opencv2/core.hpp>)
  #define MOVUTL_HAS_OPENCV
#endif
// clang-format on
