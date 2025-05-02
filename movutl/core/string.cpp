#include <movutl/core/string.hpp>

namespace mu {

bool fuzzy_match(const char* src, const char* filter) {
  if(!src || !filter) return true;
  while(*filter) {
    char c = *filter++;
    src = std::strchr(src, c);
    if(!src) return false;
    src++;
  }
  return true;
}

} // namespace mu
