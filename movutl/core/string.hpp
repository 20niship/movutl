#pragma once
#include <cstdlib>
#include <cstring>
#include <movutl/core/assert.hpp>
#include <new>
#include <vector>

namespace mu {


class CStr {
private:
  char* data = nullptr;
  size_t size_ = 0;

public:
  CStr() : data(nullptr) {}
  explicit CStr(const char* str) {
    if(!str) {
      clear();
      return;
    }
    size_ = strlen(str);
    data = static_cast<char*>(malloc(size_ + 1));
    if(!data) throw std::bad_alloc();
    strncpy(data, str, size_);
  }
  explicit CStr(const char* str, size_t len) {
    if(!str) {
      clear();
      return;
    }
    size_ = len;
    data = static_cast<char*>(malloc(size_ + 1));
    if(!data) throw std::bad_alloc();
    strncpy(data, str, size_);
  }
  explicit CStr(const std::string& str) : CStr(str.c_str()) {}
  CStr(const CStr& other) : data(nullptr) {
    if(!other.data) {
      clear();
      return;
    }
    size_ = strlen(other.data);
    data = static_cast<char*>(malloc(size_ + 1));
    if(!data) throw std::bad_alloc();
    strncpy(data, other.data, size_);
  }

  CStr(CStr&& other) noexcept : data(other.data) { other.data = nullptr; }
  ~CStr() { clear(); }

  CStr& operator=(const CStr& other) {
    if(this != &other) {
      free(data);
      if(!other.data) {
        clear();
        return *this;
      }
      size_ = strlen(other.data);
      data = static_cast<char*>(malloc(size_ + 1));
      if(!data) throw std::bad_alloc();
      strncpy(data, other.data, size_);
    }
    return *this;
  }
  bool empty() const { return !data || size_ == 0; }
  void clear() {
    if(data) free(data);
    data = nullptr;
    size_ = 0;
  }

  CStr& operator=(CStr&& other) noexcept {
    if(this != &other) {
      free(data);
      data = other.data;
      other.data = nullptr;
    }
    return *this;
  }

  size_t length() const { return size_; }
  size_t size() const { return size_; }

  char& operator[](size_t index) {
    MU_ASSERT(data);
    MU_ASSERT(index < length());
    return data[index];
  }

  const char& operator[](size_t index) const {
    MU_ASSERT(data);
    MU_ASSERT(index < length());
    return data[index];
  }

  CStr substr(size_t pos, size_t len) const {
    MU_ASSERT(data);
    MU_ASSERT(pos < length());
    len = std::min(len, length() - pos);
    char* buffer = static_cast<char*>(malloc(len + 1));
    if(!buffer) throw std::bad_alloc();
    strncpy(buffer, data + pos, len);
    buffer[len] = '\0';
    CStr result(buffer);
    free(buffer);
    return result;
  }

  size_t find(const char* str) const {
    if(!data || !str) return std::string::npos;
    const char* found = strnstr(data, str, length());
    return found ? static_cast<size_t>(found - data) : std::string::npos;
  }

  bool contains(const char* str) const { return find(str) != std::string::npos; }

  std::vector<CStr> split(const char delimiter) const {
    std::vector<CStr> result;
    if(!data) return result;
    const char* start = data;
    const char* end = nullptr;
    while((end = strchr(start, delimiter)) != nullptr) {
      size_t len = static_cast<size_t>(end - start);
      char* buffer = static_cast<char*>(malloc(len + 1));
      if(!buffer) throw std::bad_alloc();
      strncpy(buffer, start, len);
      buffer[len] = '\0';
      result.emplace_back(buffer);
      free(buffer);
      start = end + 1;
    }
    if(*start) result.emplace_back(start);
    return result;
  }

  bool operator==(const CStr& other) const {
    if(!data || !other.data) return data == other.data;
    return strcmp(data, other.data) == 0;
  }
  bool operator!=(const CStr& other) const { return !(*this == other); }

  CStr operator+(const CStr& other) const {
    size_t len1 = length();
    size_t len2 = other.length();
    char* buffer = static_cast<char*>(malloc(len1 + len2 + 1));
    if(!buffer) throw std::bad_alloc();
    if(data) strcpy(buffer, data);
    if(other.data) strcat(buffer, other.data);
    CStr result(buffer);
    free(buffer);
    return result;
  }

  CStr& operator+=(const CStr& other) {
    *this = *this + other;
    return *this;
  }
  const char* c_str() const { return data; }
  void to_upper() {
    if(!data) return; // 空文字列は処理しない
    for(size_t i = 0; i < length(); ++i) {
      if('a' <= data[i] && data[i] <= 'z') data[i] -= ('a' - 'A');
    }
  }
  void to_lower() {
    if(!data) return; // 空文字列は処理しない
    for(size_t i = 0; i < length(); ++i) {
      if('A' <= data[i] && data[i] <= 'Z') data[i] += ('a' - 'A');
    }
  }

  void replace(const char* target, const char* replacement) {
    if(!data || !target || !replacement) return;
    size_t target_len = strlen(target);
    size_t replacement_len = strlen(replacement);
    size_t pos = 0;
    while((pos = find(target)) != std::string::npos) {
      size_t new_size = size_ - target_len + replacement_len;
      char* buffer = static_cast<char*>(malloc(new_size + 1));
      if(!buffer) throw std::bad_alloc();
      strncpy(buffer, data, pos);
      strncpy(buffer + pos, replacement, replacement_len);
      strncpy(buffer + pos + replacement_len, data + pos + target_len, size_ - pos - target_len);
      buffer[new_size] = '\0';
      free(data);
      data = buffer;
      size_ = new_size;
    }
  }
};

template <size_t N = 64> struct FixStringBase {
  char data[N];
  FixStringBase() { data[0] = '\0'; }
  FixStringBase(const char* str) {
    int l = std::min(strlen(str), N);
    strncpy(data, str, l);
    data[l] = '\0';
  }
  FixStringBase(const std::string& str) : FixStringBase(str.c_str()) {}
  FixStringBase(const CStr& str) : FixStringBase(str.c_str()) {}
  FixStringBase(const FixStringBase& other) : FixStringBase(other.data) {}
  FixStringBase(FixStringBase&& other) noexcept : FixStringBase(other.data) {}
  FixStringBase& operator=(const FixStringBase& other) {
    int l = std::min(strlen(other.data), N);
    if(this != &other) strncpy(data, other.data, l);
    data[l] = '\0';
    return *this;
  }
  const char* c_str() const { return data; }
  size_t size() { return N; }
  char operator[](size_t index) const {
    MU_ASSERT(index < N);
    return data[index];
  }
  char& operator[](size_t index) {
    MU_ASSERT(index < N);
    return data[index];
  }
  bool empty() const { return data[0] == '\0'; }
  bool operator==(const FixStringBase& other) const { return strncmp(data, other.data, N) == 0; }
};

typedef FixStringBase<64> FixString;

bool fuzzy_match(const char* src, const char* filter);

} // namespace mu
