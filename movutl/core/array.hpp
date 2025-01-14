#pragma once
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <type_traits>

namespace mu {

template <typename T, size_t N> struct Array {
  T data[N];

  Array() = default;
  Array(const Array& a) {
    for(size_t i = 0; i < N; ++i) data[i] = a.data[i];
  }

  size_t size() const { return N; }
  T* begin() { return data; }
  T* end() { return data + N; }
  const T* begin() const { return data; }
  const T* end() const { return data + N; }
  T& operator[](size_t i) { return data[i]; }
  const T& operator[](size_t i) const { return data[i]; }

  bool contains(const T& value) const {
    if constexpr(std::is_same_v<T, const char*>) {
      for(size_t i = 0; i < N; ++i)
        if(strcmp(data[i], value) == 0) return true;
      return false;
    } else {
      for(size_t i = 0; i < N; ++i)
        if(data[i] == value) return true;
    }
    return false;
  }
};

} // namespace mu
