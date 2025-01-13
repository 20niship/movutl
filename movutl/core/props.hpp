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
  PropT_Vec2,      //
  PropT_Vec3,      //
  PropT_Vec4,      //
  PropT_Entity,    //
  PropT_Color,     //
  PropT_Undefined  //
};

struct Props {
public:
  using Value = std::variant<int, float, std::string, bool, Vec2, Vec3, Vec4, Vec4b, Entity*, Props>;
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
  template <typename T> bool has(const std::string& key) const {
    if(!values.contains(key)) return false;
    return std::holds_alternative<T>(values.at(key));
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
    if(is_type<Vec2>(key)) return PropT_Vec2;
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
  union PropUnion {
    FloatProp float_;
    int int_;
    bool bool_;
    char str_[255];
    Vec2 vec2_;
    Vec3 vec3_;
    Vec4 vec4_;
    Vec4b color_;
    SelectionProp selection_;
    constexpr PropUnion() : float_() {}
  };
  PropUnion value_;
  PropType type = PropT_Undefined;

public:
  std::string name;     // getProp / setPropで使用する名前と同系列のもの
  std::string dispname; // インスペクター画面などに表示するためのもの
  std::string categ;
  std::string desc;
  float min = 0.0f;
  float max = 0.0f;
  float step = 1.0f;
  bool readonly = false;

  PropInfoBase() = default;
  ~PropInfoBase() = default;

  // clang-format off
  template <typename T> T get() const {
    if constexpr(std::is_same_v<T, float>) return value_.float_;
    if constexpr(std::is_same_v<T, int>) return value_.int_;
    if constexpr(std::is_same_v<T, bool>) return value_.bool_;
    if constexpr(std::is_same_v<T, std::string>) return value_.str_;
    if constexpr(std::is_same_v<T, Vec2>) return value_.vec3_;
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
    if constexpr(std::is_same_v<T, Vec2>) {value_.vec3_ = value; type = PropT_Vec2; return *this;}
    if constexpr(std::is_same_v<T, Vec3>) {value_.vec3_ = value; type = PropT_Vec3; return *this; }
    if constexpr(std::is_same_v<T, Vec4>) {value_.vec4_ = value; type = PropT_Vec4; return *this; }
    if constexpr(std::is_same_v<T, Vec4b>) {value_.color_ = value; type = PropT_Color; return *this; }
    MU_FAIL("Invalid type");
    return *this;
  }
  // clang-format on
  PropType getType() const { return type; }
};

struct PropsInfo {
  std::vector<PropInfoBase> props;
  size_t size() const { return props.size(); }
  bool empty() const { return props.empty(); }
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
  PropInfoBase get(const std::string& key) const {
    for(auto& p : props)
      if(p.name == key) return p;
    return PropInfoBase();
  }
  void set(const std::string& key, const PropInfoBase& value) {
    for(auto& p : props)
      if(p.name == key) p.set(value);
    props.push_back(value);
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

  void add_float_prop(                                   //
    const char* name, const char* cat, const char* desc, //
    float def, float min, float max,                     //
    float step = 1.0, bool is_angle = false, bool is_radian = false);

  void add_int_prop(const char* name, const char* cat, const char* desc, int def, int min, int max, int step = 1);
  void add_bool_prop(const char* name, const char* category, const char* desc, bool def);
  void add_string_prop(const char* name, const char* cat, const char* desc, const char* def);
  void add_path_prop(const char* name, const char* category, const char* desc, const char* def);
  void add_color_prop(const char* name, const char* category, const char* desc, Vec4b def);
  void add_vec2_prop(const char* name, const char* cat, const char* desc, //
                     Vec2 def, float min, float max, float step = 0.1f);
  void add_vec3_prop(const char* name, const char* cat, const char* desc, //
                     Vec3 def, float min, float max, float step = 0.1f);
  void add_selection_prop(const char* name, const char* cat, const char* desc, //
                          const char* items[], uint8_t count, uint8_t def);

  void set_last_prop_dispname(const char* dispname);
  void set_last_prop_category(const char* category);
  void set_last_prop_desc(const char* desc);
  void set_last_prop_readonly(bool readonly);
};


} // namespace mu
