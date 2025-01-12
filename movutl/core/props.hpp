#pragma once
#include <map>
#include <movutl/core/vector.hpp>
#include <string>
#include <variant>

namespace mu {

class Entity;

enum PropType {
  PropT_Float,     //
  PropT_Int,       //
  PropT_Bool,      //
  PropT_String,    //
  PropT_Path,      //
  PropT_Selection, //
  PropT_Vec3,      //
  PropT_Vec4,      //
  PropT_Entity,    //
  PropT_Color,     //
  PropT_Undefined  //
};

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

  PropType type(const std::string& key) const {
    if(is_type<float>(key)) return PropT_Float;
    if(is_type<int>(key)) return PropT_Int;
    if(is_type<bool>(key)) return PropT_Bool;
    if(is_type<std::string>(key)) return PropT_String;
    if(is_type<Vec3>(key)) return PropT_Vec3;
    if(is_type<Vec4b>(key)) return PropT_Color;
    if(is_type<Entity*>(key)) return PropT_Entity;
    return PropT_Undefined;
  }
};

struct PropInfoBase {
  struct FloatProp {
    float default_ = 0.5f;
    bool is_angle = false;
    bool is_radian = false;
    FloatProp() = default;
    FloatProp(float def) : default_(def) {}
    ~FloatProp() = default;
  };
  struct SelectionProp {
    SelectionProp() = default;
    char items[20][10];
    uint8_t count = 0;
    uint8_t default_ = 0;
  };
  union PropUnion{
    FloatProp float_;
    int int_;
    bool bool_;
    char str_[255];
    Vec3 vec3_;
    Vec4 vec4_;
    Vec4b color_;
    SelectionProp selection_;
    constexpr PropUnion() : float_() {}
  };
  PropUnion value_;
  PropType type = PropT_Undefined;

public:
  std::string name;
  std::string categ;
  std::string desc;
  float min = 0.0f;
  float max = 1.0f;
  float step = 0.1f;

  PropInfoBase() = default;
  ~PropInfoBase() = default;

  // clang-format off
  template <typename T> T get() const {
    if constexpr(std::is_same_v<T, float>) return value_.float_;
    if constexpr(std::is_same_v<T, int>) return value_.int_;
    if constexpr(std::is_same_v<T, bool>) return value_.bool_;
    if constexpr(std::is_same_v<T, std::string>) return value_.str_;
    if constexpr(std::is_same_v<T, Vec3>) return value_.vec3_;
    if constexpr(std::is_same_v<T, Vec4>) return value_.vec4_;
    if constexpr(std::is_same_v<T, Vec4b>) return value_.color_;
    MU_ASSERT(false);
  }
  template <typename T> PropInfoBase set(const T& value) {
    if constexpr(std::is_same_v<T, float>) {value_.float_ = value; type = PropT_Float; return *this; }
    if constexpr(std::is_same_v<T, int>) {value_.int_ = value; type = PropT_Int; return *this; }
    if constexpr(std::is_same_v<T, bool>) {value_.bool_ = value; type = PropT_Bool; return *this; }
    if constexpr(std::is_same_v<T, std::string>) {value_.str_ = value; type = PropT_String; return *this; }
    if constexpr(std::is_same_v<T, Vec3>) {value_.vec3_ = value; type = PropT_Vec3; return *this; }
    if constexpr(std::is_same_v<T, Vec4>) {value_.vec4_ = value; type = PropT_Vec4; return *this; }
    if constexpr(std::is_same_v<T, Vec4b>) {value_.color_ = value; type = PropT_Color; return *this; }
    MU_ASSERT(false);
  }
  // clang-format on
  PropType getType() const { return type; }
};

struct PropsInfo {
  std::vector<PropInfoBase> props;
  size_t size() const { return props.size(); }
  bool contains(const std::string& key) const {
    for(auto& p : props)
      if(p.name == key) return true;
    return false;
  }
  std::vector<std::string> keys() const {
    std::vector<std::string> keys;
    for(auto& p : props) keys.push_back(p.name);
    return keys;
  }
  template <typename T> T get(const std::string& key) const {
    for(auto& p : props)
      if(p.name == key) return p.get<T>();
    return T();
  }
  template <typename T> void set(const std::string& key, const T& value) {
    for(auto& p : props)
      if(p.name == key) p.set(value);
  }

  PropInfoBase& operator[](size_t idx) { return props[idx]; }
  const PropInfoBase& operator[](size_t idx) const { return props[idx]; }
  PropInfoBase& operator[](const std::string& key) {
    for(auto& p : props)
      if(p.name == key) return p;
    MU_FAIL("[PropsInfoBase] key not found");
    return props[0];
  }
  const PropInfoBase& operator[](const std::string& key) const {
    for(auto& p : props)
      if(p.name == key) return p;
    MU_FAIL("[PropsInfoBase] key not found");
    return props[0];
  }
};


} // namespace mu
