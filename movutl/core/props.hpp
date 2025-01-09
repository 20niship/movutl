#pragma once
#include <map>
#include <movutl/core/vector.hpp>
#include <string>
#include <variant>

namespace mu {

class Entity;

struct Props {
public:
  using Value = std::variant<int, float, std::string, bool, Vec2, Vec3, Vec4b, Entity*, Props>;
  std::map<std::string, Value> values;

  bool contains(const std::string& key) const { return values.contains(key); }
  template <typename T> bool is_type(const std::string& key) const {
    if(!values.contains(key)) return false;
    return std::holds_alternative<T>(values.at(key));
  }
  template <typename T> T get(const std::string& key) const {
    MU_ASSERT(values.contains(key));
    auto it = values.find(key);
    MU_ASSERT(std::holds_alternative<T>(it->second));
    return std::get<T>(it->second);
  }

  template <typename T> void set(const std::string& key, const T& value) { values[key] = value; }
  template <typename T> T get_or(const std::string& key, const T& value) const {
    if(values.contains(key) && std::holds_alternative<T>(values.at(key))) return std::get<T>(values.at(key));
    return value;
  }

  void push_back(const Props::Value& p) {
    std::string key = std::to_string(values.size());
    values[key] = p;
  }

  Value operator[](const std::string& key) const { return values.at(key); }
  Value& operator[](const std::string& key) { return values[key]; }

  bool is_array() const {
    try {
      for(auto& [k, _] : values) std::stoi(k);
    } catch(...) {
      return false;
    }
    return true;
  }

  static Props LoadJson(const std::string& filename);
  static Props LoadJsonFile(const std::string& filename);
  std::string dump_json(int indent = 2) const;
  bool dump_json_file(const std::string& filename, int indent = 2) const;
};

struct PropInfoBase {
public:
  std::string category;
  std::string description;
};

struct FloatProp final : public PropInfoBase {
public:
  float min = 0.0f;
  float max = 1.0f;
  float step = 0.1f;
  float default_ = 0.5f;
};

struct IntProp final : public PropInfoBase {
public:
  int min = 0;
  int max = 100;
  int step = 1;
  int default_ = 50;
};

struct BoolProp final : public PropInfoBase {
public:
  bool default_ = false;
};

struct StringProp final : public PropInfoBase {
public:
  std::string default_;
};

using PropInfos = std::map<std::string, std::variant<FloatProp, IntProp, BoolProp, StringProp>>;

} // namespace mu
