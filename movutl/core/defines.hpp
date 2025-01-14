#pragma once
#include <movutl/core/platform.hpp>

#ifndef MU_API
#define MU_API
#endif

// clang-format off
#define MOVUTL_DECLARE_SINGLETON(className) \
protected: \
  static className* singleton_; \
public: \
  static className* Get() { if(!singleton_) singleton_ = new className(); return singleton_; } \
  className(const className&) = delete; \
  className& operator=(const className&) = delete; \
  className(className&&) = delete; \
  className& operator=(className&&) = delete; 


// https://jumble-note.blogspot.com/2017/12/c.html
#define MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS(T)                                 \
constexpr T operator|(const T lhs, const T rhs)                             \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return static_cast<T>(static_cast<U>(lhs) | static_cast<U>(rhs));       \
}                                                                           \
                                                                            \
constexpr T operator&(const T lhs, const T rhs)                             \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return static_cast<T>(static_cast<U>(lhs) & static_cast<U>(rhs));       \
}                                                                           \
                                                                            \
constexpr T operator^(const T lhs, const T rhs)                             \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return static_cast<T>(static_cast<U>(lhs) ^ static_cast<U>(rhs));       \
}                                                                           \
                                                                            \
constexpr T operator~(const T val)                                          \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return static_cast<T>(~static_cast<U>(val));                            \
}                                                                           \
                                                                            \
inline T& operator|=(T& lhs, const T& rhs)                                  \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return lhs = static_cast<T>(static_cast<U>(lhs) | static_cast<U>(rhs)); \
}                                                                           \
                                                                            \
inline T& operator&=(T& lhs, const T& rhs)                                  \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return lhs = static_cast<T>(static_cast<U>(lhs) & static_cast<U>(rhs)); \
}                                                                           \
                                                                            \
inline T& operator^=(T& lhs, const T& rhs)                                  \
{                                                                           \
    using U = typename std::underlying_type<T>::type;                       \
    return lhs = static_cast<T>(static_cast<U>(lhs) ^ static_cast<U>(rhs)); \
}


// https://stackoverflow.com/questions/525677/is-there-a-way-to-disable-all-warnings-with-a-pragma
//
//

#if defined(HEDLEY_DIAGNOSTIC_PUSH)
#  undef HEDLEY_DIAGNOSTIC_PUSH
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  define MOVUTL_WARNING_PUSH  _Pragma("warning(push)")
#  define MOVUTL_DISABLE_ALL_WARNINGS _Pragma("warning (disable : 4820 4619 4668 4710 4711 4820 4514 4625 4626 5026 5027 4061 4365 4571 4623 4625 4626 4628 4640 4668 4710 4711 4820 5026 5027 5039 5045 4061 4365 4571 4623 4625 4626 4628 4640 4668 4710 4711 4820 5026 5027 5039 5045)")
#  define MOVUTL_WARNING_POP  _Pragma("warning (pop)")
#elif defined(__clang__)
#  define MOVUTL_WARNING_PUSH  _Pragma("clang diagnostic push")
#  define MOVUTL_DISABLE_ALL_WARNINGS  \
    _Pragma("clang diagnostic ignored \"-Wunused-variable\"") \
    _Pragma("clang diagnostic ignored \"-Wignored-qualifiers\"") \
    _Pragma("clang diagnostic ignored \"-Wunused-parameter\"") \
    _Pragma("clang diagnostic ignored \"-Wunused-function\"") \
    _Pragma("clang diagnostic ignored \"-Wunused-but-set-variable\"")
    _Pragma("clang diagnostic ignored \"-Wunused-value\"") \
    _Pragma("clang diagnostic ignored \"-Wmissing-field-initializers\"") \
    _Pragma("clang diagnostic ignored \"-Wunknown-pragmas\"") \
    _Pragma("clang diagnostic ignored \"-Wdeprecated\"") \
    _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"") \
    _Pragma("clang diagnostic ignored \"-Wsign-conversion\"") \
    _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"") \
    _Pragma("clang diagnostic ignored \"-Wmacro-redefined\"") \
    _Pragma("clang diagnostic ignored \"-Wsign-compare\"") \
    _Pragma("clang diagnostic ignored \"-Wunused-lambda-capture\"") \
    _Pragma("clang diagnostic ignored \"-Wreorder\"") \
    _Pragma("clang diagnostic ignored \"-Woverloaded-virtual\"") \
    _Pragma("clang diagnostic ignored \"-Wreturn-type\"") \
    _Pragma("clang diagnostic ignored \"-Wdeprecated-non-prototype\"") \
    _Pragma("clang diagnostic ignored \"-Wtautological-constant-out-of-range-compare\"")
#  define MOVUTL_WARNING_POP  _Pragma("clang diagnostic pop")
#else 
#  define MOVUTL_WARNING_PUSH  _Pragma("GCC diagnostic push")
#  define MOVUTL_DISABLE_ALL_WARNINGS  \
  _Pragma("GCC diagnostic ignored \"-Wunused-variable\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-function\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-but-set-variable\"")
  _Pragma("GCC diagnostic ignored \"-Wunused-value\"") \
  _Pragma("GCC diagnostic ignored \"-Wignored-qualifiers\"") \
  _Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"") \
  _Pragma("GCC diagnostic ignored \"-Wunknown-pragmas\"") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated\"") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"") \
  _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"") \
  _Pragma("GCC diagnostic ignored \"-Wstringop-truncation\"") \
  _Pragma("GCC diagnostic ignored \"-Wsign-compare\"") \
  _Pragma("GCC diagnostic ignored \"-Wredundant-move\"") \
  _Pragma("GCC diagnostic ignored \"-Warray-bounds\"") \
  _Pragma("GCC diagnostic ignored \"-Woverloaded-virtual\"") \
  _Pragma("GCC diagnostic ignored \"-Wreorder\"") \
  _Pragma("GCC diagnostic ignored \"-Wreturn-type\"") \
  _Pragma("GCC diagnostic ignored \"-Wextra\"") \
  _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"") \
  _Pragma("GCC diagnostic ignored \"-Wcomment\"")
#  define MOVUTL_WARNING_POP  _Pragma("GCC diagnostic pop")
#endif


#ifndef MU_API
#ifdef MOVUTL_PLATFORM_WINDOWS
#  ifndef BUILD_MOVUTL
#    define MU_API __declspec(dllexport)
#    define MU_API_FUNC extern "C" __declspec(dllexport)
#  else
#    define MU_API __declspec(dllimport)
#    define MU_API_FUNC extern "C" __declspec(dllimport)
#  endif
#else
#  define MU_API
#  define MU_API_FUNC extern "C"
#endif // MOVUTL_PLATFORM_WINDOWS
#endif // MU_API

// https://gist.github.com/yossi-tahara/06b42fd6c1d5f2dae9b196fba9c33052
// https://qiita.com/Chironian/items/3fb61cffa2a20dbee5c2
#ifdef MOVUTL_DLL_BODY
    #ifdef __WIN32__
        #define MOVUTL_DLL_EXPORT __declspec(dllexport)
    #else
        #define MOVUTL_DLL_EXPORT __attribute__((visibility ("default")))
    #endif
#else
    #define MOVUTL_DLL_EXPORT
#endif

#ifndef MOVUTL_UNUSED
#define MOVUTL_UNUSED(x) (void)(x)
#endif

// clang-format on
