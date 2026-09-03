#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cutil/prop.hpp>
#include <movutl/core/prop_types.hpp>
#include <type_traits>
#include <variant>

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
  uint32_t frame_    = 0;
  float ease_        = 0;
  float ease2_       = 0;
  AniInterpType type = AniInterpType::LINEAR;
  AnimKeyframe()     = default;
  AnimKeyframe(T value) { value_ = value; }
};

namespace detail {
// t(0-1)にイージングを適用する。Customはease_/ease2_を使ったベジェ補間が本来必要だが未実装のためlinearとして扱う
inline double apply_ease(AniInterpType type, double t) {
  switch(type) {
    case EaseIn:
    case EaseInQuad: return t * t;
    case EaseOut:
    case EaseOutQuad: return 1.0 - (1.0 - t) * (1.0 - t);
    case EaseInOut:
    case EaseInOutQuad: return t < 0.5 ? 2.0 * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 2) / 2.0;
    case EaseInCubic: return t * t * t;
    case EaseOutCubic: return 1.0 - std::pow(1.0 - t, 3);
    case EaseInOutCubic: return t < 0.5 ? 4.0 * t * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 3) / 2.0;
    case LINEAR:
    case Custom:
    default: return t;
  }
}

// bool/std::string/Entity*は数値演算が無いため補間できず、区間開始側のキーフレームの値をそのまま返す(ステップ補間)
template <typename T> T anim_lerp(const T& a, const T& b, double t) {
  if constexpr(std::is_same_v<T, bool> || std::is_same_v<T, std::string> || std::is_same_v<T, Entity*>) {
    MU_UNUSED(b);
    MU_UNUSED(t);
    return a;
  } else {
    return (T)(a + (b - a) * t);
  }
}
} // namespace detail

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
    keys.push_back(AnimKeyframe<T>()); // デフォルト値がvalue_=T(), frame_=0, type=LINEARなのでそのまま使う
  }
  T get(uint32_t frame) const {
    if(keys.empty()) return T();
    if(keys.size() == 1 || frame <= keys.front().frame_) return keys.front().value_;
    if(frame >= keys.back().frame_) return keys.back().value_;

    // frame以下の最後のキー(k0)とその次のキー(k1)を探す
    size_t i = 0;
    while(i + 1 < keys.size() && keys[i + 1].frame_ <= frame) i++;
    const auto& k0 = keys[i];
    const auto& k1 = keys[i + 1];
    if(k1.frame_ == k0.frame_) return k1.value_;
    double t = (double)(frame - k0.frame_) / (double)(k1.frame_ - k0.frame_);
    t        = detail::apply_ease(k0.type, t);
    return detail::anim_lerp(k0.value_, k1.value_, t);
  }
  // frameに対応するキーフレームが既にあれば値を更新、無ければ挿入する(frame昇順を維持)
  bool add_keyframe(uint32_t frame, T value, AniInterpType t = AniInterpType::LINEAR) {
    for(auto& k : keys) {
      if(k.frame_ == frame) {
        k.value_ = value;
        k.type   = t;
        return true;
      }
    }
    AnimKeyframe<T> kf;
    kf.value_ = value;
    kf.frame_ = frame;
    kf.type   = t;
    auto it   = std::lower_bound(keys.begin(), keys.end(), frame, [](const AnimKeyframe<T>& k, uint32_t f) { return k.frame_ < f; });
    keys.insert(it, kf);
    return true;
  }
  bool has_animation() const { return keys.size() > 1; }
  void clear() { keys.clear(); }
  const cutil::PropInfo* get_type() const { return cutil::prop_info_of<T>(); }
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
  cutil::Prop get(uint32_t frame) const;

  const cutil::PropInfo* get_type(int index) const {
    if(index < 0 || index >= (int)props.size()) return nullptr;
    return std::visit([](auto&& arg) { return arg.get_type(); }, props[index]);
  }

  template <typename T> T get(int index, uint32_t frame = 0) {
    if(index < 0 || index >= (int)props.size()) return T();

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

  // 指定frameにキーフレームが無ければ追加、既にあれば更新する。has_animation()がfalse(単一キーのみ)の場合はそのキーの値を直接書き換える(常時アニメーション化を避けるため)
  template <typename T> void set_value(int idx, uint32_t frame, T value) {
    if(idx < 0 || idx >= (int)props.size()) return;
    if(std::holds_alternative<PAniClip<T>>(props[idx])) {
      auto& clip = std::get<PAniClip<T>>(props[idx]);
      if(clip.keys.empty()) {
        clip.keys.push_back(AnimKeyframe<T>(value));
      } else if(!clip.has_animation()) {
        clip.keys[0].value_ = value;
        clip.keys[0].frame_ = frame;
      } else {
        clip.add_keyframe(frame, value);
      }
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

  void add_props(const cutil::Prop& defaults);
};

} // namespace mu
