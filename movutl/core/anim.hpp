#pragma once
#include <cstdint>
#include <movutl/core/props.hpp>

namespace mu {

enum AniInterpType {
  LINEAR,
  EaseIn,
  EaseOut,
  EaseInOut,

  EaseInQuad,
  EaseOutQuad,
  EaseInOutQuad,

  EaseInCubic,
  EaseOutCubic,
  EaseInOutCubic,
  Custom,
};

template <typename T> struct AnimKeyframe {
  T value_;
  uint32_t frame_ = 0;
  float ease_ = 0;
  float ease2_ = 0;
  AniInterpType type = AniInterpType::LINEAR;
  AnimKeyframe() = default;
  AnimKeyframe(T value) { value_ = value; }
};

template <typename T> struct PAniClip {
public:
  std::vector<AnimKeyframe<T>> keys;
  std::string keyname;

  PAniClip() { reset(); }
  PAniClip(const std::string& name, const T& value) {
    keyname = name;
    keys.push_back(AnimKeyframe<T>(value));
  }
  void reset() {
    keys.clear();
    keys.push_back({T(), 0, 0, 0, AniInterpType::LINEAR});
  }
  T get(uint32_t frame) const {
    MU_UNUSED(frame);
    return get_first(); // TODO : get value by frame
  }
  bool add_keyframe(uint32_t frame, T value, AniInterpType t = AniInterpType::LINEAR);
  bool has_animation() const { return keys.size() > 1; }
  void clear() { keys.clear(); }
  PropType get_type() const {
    // clang-format off
    if constexpr(std::is_same_v<T, float> || std::is_same_v<T, double>) return PropT_Float;
    else if constexpr(std::is_integral_v<T>) return PropT_Int;
    else if constexpr(std::is_same_v<T, std::string>) return PropT_String;
    else if constexpr(std::is_same_v<T, bool>) return PropT_Bool;
    else if constexpr(std::is_same_v<T, Vec2>) return PropT_Vec2;
    else if constexpr(std::is_same_v<T, Vec3>) return PropT_Vec3;
    else if constexpr(std::is_same_v<T, Vec4>) return PropT_Vec4;
    else if constexpr(std::is_same_v<T, Vec4b>) return PropT_Color;
    else if constexpr(std::is_same_v<T, Entity*>) return PropT_Entity;
    // clang-format on
  }
  T get_first() const {
    if(keys.empty()) return T();
    return keys[0].value_;
  }
};

struct AnimProps {
public:
  // clang-format off
  using Types = std::variant< \
        PAniClip<int>,  \
        PAniClip<float>,  \
        PAniClip<std::string>, \
        PAniClip<bool>, \
        PAniClip<Vec2>, \
        PAniClip<Vec3>, \
        PAniClip<Vec4>, \
        PAniClip<Vec4b>,  \
        PAniClip<Entity*>\
    >;
  // clang-format on

  std::vector<Types> props;
  Props get(uint32_t frame);

  PropType get_type(int index) const {
    if(index < 0 || index >= (int)props.size()) return PropT_Undefined;
    return std::visit([](auto&& arg) { return arg.get_type(); }, props[index]);
  }

  template <typename T> T get(int index) {
    if(index < 0 || index >= (int)props.size()) return T();
    int frame = 0; // TODO

    // clang-format off
    if constexpr(std::is_same_v<T, float> || std::is_same_v<T, double>) return std::get<PAniClip<float>>(props[index]).get(frame);
    else if constexpr(std::is_integral_v<T>) return (int)std::get<PAniClip<int>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, std::string>) return (std::string)std::get<PAniClip<std::string>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, bool>) return (bool)std::get<PAniClip<bool>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, Vec2>) return std::get<PAniClip<Vec2>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, Vec3>) return std::get<PAniClip<Vec3>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, Vec4>) return std::get<PAniClip<Vec4>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, Vec4b>) return std::get<PAniClip<Vec4b>>(props[index]).get(frame);
    else if constexpr(std::is_same_v<T, Entity*>) return std::get<PAniClip<Entity*>>(props[index]).get(frame);
    // clang-format on
  }

  bool contains(const std::string& name) const {
    for(auto& prop : props)
      if(std::visit([&name](auto&& arg) { return arg.keyname == name; }, prop)) return true;
    return false;
  }
  bool erase(const std::string& name) {
    for(auto it = props.begin(); it != props.end(); it++) {
      if(std::visit([&name](auto&& arg) { return arg.keyname == name; }, *it)) {
        props.erase(it);
        return true;
      }
    }
    return false;
  }
  size_t size() const { return props.size(); }
  template <typename T> void add_prop(const std::string& name, T value) {
    // clang-format off
    if constexpr(std::is_same_v<T, float> || std::is_same_v<T, double>) props.push_back(PAniClip<float>(name, value));
    else if constexpr(std::is_integral_v<T>) props.push_back(PAniClip<int>(name, value));
    else if constexpr(std::is_same_v<T, std::string>) props.push_back(PAniClip<std::string>(name, value));
    else if constexpr(std::is_same_v<T, bool>) props.push_back(PAniClip<bool>(name, value));
    else if constexpr(std::is_same_v<T, Vec2>) props.push_back(PAniClip<Vec2>(name, value));
    else if constexpr(std::is_same_v<T, Vec3>) props.push_back(PAniClip<Vec3>(name, value));
    else if constexpr(std::is_same_v<T, Vec4b>) props.push_back(PAniClip<Vec4b>(name, value));
    else if constexpr(std::is_same_v<T, Entity*>) props.push_back(PAniClip<Entity*>(name, value));
    // clang-format on
  }

  template <typename T> void set_value(int idx, uint32_t frame, T value) {
    MU_UNUSED(frame); // TODO
    if(idx < 0 || idx >= (int)props.size()) return;
    if(std::holds_alternative<PAniClip<T>>(props[idx])) {
      auto& clip = std::get<PAniClip<T>>(props[idx]);
      if(clip.keys.empty())
        clip.keys.push_back(AnimKeyframe<T>(value));
      else
        clip.keys[0].value_ = value;
    }
  }

  // operators
  /* bool operator==(const AnimProps& rhs) const { */
  /*   if(props.size() != rhs.props.size()) return false; */
  /*   for(size_t i = 0; i < props.size(); i++) { */
  /*     if(props[i] != rhs.props[i]) return false; */
  /*   } */
  /*   return true; */
  /* } */
  /* bool operator!=(const AnimProps& rhs) const { return !(*this == rhs); } */
  Types& operator[](size_t idx) { return props[idx]; }
  const Types& operator[](size_t idx) const { return props[idx]; }

  void add_props(const Props& props);
};

} // namespace mu
