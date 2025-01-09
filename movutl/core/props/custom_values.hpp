#pragma once
#include <movutl/core/props.hpp>
#include <movutl/core/vector.hpp>
#include <optional>
#include <regex>

namespace mu {

inline std::string NUM_REGEX = R"(([0-9\-\.]+))";

inline std::string getVectorPattern(int N) {
  MU_ASSERT(N > 0);
  std::string pattern(R"(\$V)");
  pattern += std::to_string(N);
  pattern += R"(\()";
  for(int i = 0; i < N; i++) {
    pattern += NUM_REGEX;
    if(i != N - 1) pattern += " ";
  }
  pattern += R"(\))";
  return pattern;
}

template <int N> inline _Vec<float, N> get_vec(const std::string& str) {
  std::regex patternReg(getVectorPattern(N));

  std::cmatch matches;
  if(!std::regex_search(str.c_str(), matches, patternReg)) return {};

  if(matches.size() != N + 1) return {};

  _Vec<float, N> v;
  for(int i = 0; i < N; i++) v[i] = (std::stof(matches[i + 1]));
  return v;
}

template <int N> inline bool is_vector(const std::string& str) {
  std::regex patternReg(getVectorPattern(N));
  std::cmatch matches;
  if(!std::regex_search(str.c_str(), matches, patternReg)) return false;
  if(matches.size() != N + 1) return false;
  for(int i = 1; i < matches.size(); i++) {
    try {
      [[maybe_unused]] float f = std::stof(matches[i]);
    } catch(...) {
      return false;
    }
  }
  return true;
}

inline std::optional<Vec4b> get_color_rgba_(const std::string& str) {
  std::cmatch matches;
  std::string colorPattern(R"(\$Col4\()");
  colorPattern += NUM_REGEX + " " + NUM_REGEX + " " + NUM_REGEX + " " + NUM_REGEX + R"(\))";
  std::regex patternReg(colorPattern);

  if(!std::regex_search(str.c_str(), matches, patternReg)) return std::nullopt;
  if(matches.size() != 5) return std::nullopt;

  try {
    Vec4b color{(uint8_t)std::stoi(matches[1]), (uint8_t)std::stoi(matches[2]), (uint8_t)std::stoi(matches[3]), (uint8_t)std::stoi(matches[4])};
    return color;
  } catch(...) {
    return std::nullopt;
  }
}

template <int N> std::string dumpjson(const _Vec<float, N>& v) {
  std::string str = "$V" + std::to_string(N) + "(";
  for(int i = 0; i < N; i++) {
    str += std::to_string(v[i]);
    if(i != N - 1) str += " ";
  }
  str += ")";
  return str;
}
template <int N> std::string dumpjson(const _Vec<uint8_t, N>& v) {
  std::string str = "$Col" + std::to_string(N) + "b" + "(";
  for(int i = 0; i < N; i++) {
    str += std::to_string(v[i]);
    if(i != N - 1) str += " ";
  }
  str += ")";
  return str;
}
} // namespace mu
