#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cutil/prop.hpp>
#include <movutl/core/prop_types.hpp>
#include <type_traits>
#include <variant>

namespace mu {

enum AniInterpType {
  LINEAR,
  EaseIn, // レガシー別名(EaseInSineと同じ式)。保存済みプロジェクトの互換性のため残す
  EaseOut,
  EaseInOut,

  EaseInQuad,
  EaseOutQuad,
  EaseInOutQuad,

  EaseInCubic,
  EaseOutCubic,
  EaseInOutCubic,

  EaseInSine,
  EaseOutSine,
  EaseInOutSine,

  EaseInQuart,
  EaseOutQuart,
  EaseInOutQuart,

  EaseInQuint,
  EaseOutQuint,
  EaseInOutQuint,

  EaseInExpo,
  EaseOutExpo,
  EaseInOutExpo,

  EaseInCirc,
  EaseOutCirc,
  EaseInOutCirc,

  EaseInBack,
  EaseOutBack,
  EaseInOutBack,

  EaseInElastic,
  EaseOutElastic,
  EaseInOutElastic,

  EaseInBounce,
  EaseOutBounce,
  EaseInOutBounce,

  Custom, // 2点3次ベジエ: cubic-bezier(ease_=x1, ease2_=y1, ease3_=x2, ease4_=y2)相当
};

template <typename T> struct AnimKeyframe {
  T value_;
  uint32_t frame_    = 0;
  float ease_        = 0.42f;
  float ease2_       = 0.0f;
  float ease3_       = 0.58f;
  float ease4_       = 1.0f;
  AniInterpType type = AniInterpType::LINEAR;
  AnimKeyframe()     = default;
  AnimKeyframe(T value) { value_ = value; }
};

namespace detail {

// outBounceのみ独立実装(in/inOutはこれを反転/合成して作る、Pennerイージングの標準実装)
inline double ease_out_bounce(double t) {
  constexpr double n1 = 7.5625, d1 = 2.75;
  if(t < 1.0 / d1) return n1 * t * t;
  if(t < 2.0 / d1) {
    t -= 1.5 / d1;
    return n1 * t * t + 0.75;
  }
  if(t < 2.5 / d1) {
    t -= 2.25 / d1;
    return n1 * t * t + 0.9375;
  }
  t -= 2.625 / d1;
  return n1 * t * t + 0.984375;
}

// CSS cubic-bezier(x1,y1,x2,y2)相当。P0=(0,0),P3=(1,1)固定の3次ベジエをtでNewton-Raphson近似する
inline double cubic_bezier_component(double u, double p1, double p2) {
  double v = 1.0 - u;
  return 3.0 * v * v * u * p1 + 3.0 * v * u * u * p2 + u * u * u;
}
inline double cubic_bezier_derivative(double u, double p1, double p2) {
  double v = 1.0 - u;
  return 3.0 * v * v * p1 + 6.0 * v * u * (p2 - p1) + 3.0 * u * u * (1.0 - p2);
}
inline double eval_cubic_bezier(double x1, double y1, double x2, double y2, double t) {
  if(t <= 0.0) return 0.0;
  if(t >= 1.0) return 1.0;
  double u = t;
  for(int i = 0; i < 8; i++) {
    double dx = cubic_bezier_component(u, x1, x2) - t;
    double d  = cubic_bezier_derivative(u, x1, x2);
    if(std::abs(d) < 1e-6) break;
    u = std::clamp(u - dx / d, 0.0, 1.0);
  }
  return cubic_bezier_component(u, y1, y2);
}

// t(0-1)にイージングを適用する。Custom(ease_,ease2_,ease3_,ease4_)以外は標準Pennerイージング公式
inline double apply_ease(AniInterpType type, double t, float ease_ = 0, float ease2_ = 0, float ease3_ = 0.58f, float ease4_ = 1.0f) {
  constexpr double kPi = 3.14159265358979323846;
  switch(type) {
    case EaseIn:
    case EaseInSine: return 1.0 - std::cos(t * kPi / 2.0);
    case EaseOut:
    case EaseOutSine: return std::sin(t * kPi / 2.0);
    case EaseInOut:
    case EaseInOutSine: return -(std::cos(kPi * t) - 1.0) / 2.0;

    case EaseInQuad: return t * t;
    case EaseOutQuad: return 1.0 - (1.0 - t) * (1.0 - t);
    case EaseInOutQuad: return t < 0.5 ? 2.0 * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 2) / 2.0;

    case EaseInCubic: return t * t * t;
    case EaseOutCubic: return 1.0 - std::pow(1.0 - t, 3);
    case EaseInOutCubic: return t < 0.5 ? 4.0 * t * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 3) / 2.0;

    case EaseInQuart: return t * t * t * t;
    case EaseOutQuart: return 1.0 - std::pow(1.0 - t, 4);
    case EaseInOutQuart: return t < 0.5 ? 8.0 * t * t * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 4) / 2.0;

    case EaseInQuint: return t * t * t * t * t;
    case EaseOutQuint: return 1.0 - std::pow(1.0 - t, 5);
    case EaseInOutQuint: return t < 0.5 ? 16.0 * t * t * t * t * t : 1.0 - std::pow(-2.0 * t + 2.0, 5) / 2.0;

    case EaseInExpo: return t <= 0.0 ? 0.0 : std::pow(2.0, 10.0 * t - 10.0);
    case EaseOutExpo: return t >= 1.0 ? 1.0 : 1.0 - std::pow(2.0, -10.0 * t);
    case EaseInOutExpo:
      if(t <= 0.0) return 0.0;
      if(t >= 1.0) return 1.0;
      return t < 0.5 ? std::pow(2.0, 20.0 * t - 10.0) / 2.0 : (2.0 - std::pow(2.0, -20.0 * t + 10.0)) / 2.0;

    case EaseInCirc: return 1.0 - std::sqrt(1.0 - std::pow(t, 2));
    case EaseOutCirc: return std::sqrt(1.0 - std::pow(t - 1.0, 2));
    case EaseInOutCirc: return t < 0.5 ? (1.0 - std::sqrt(1.0 - std::pow(2.0 * t, 2))) / 2.0 : (std::sqrt(1.0 - std::pow(-2.0 * t + 2.0, 2)) + 1.0) / 2.0;

    case EaseInBack: {
      constexpr double c1 = 1.70158, c3 = c1 + 1.0;
      return c3 * t * t * t - c1 * t * t;
    }
    case EaseOutBack: {
      constexpr double c1 = 1.70158, c3 = c1 + 1.0;
      return 1.0 + c3 * std::pow(t - 1.0, 3) + c1 * std::pow(t - 1.0, 2);
    }
    case EaseInOutBack: {
      constexpr double c1 = 1.70158, c2 = c1 * 1.525;
      return t < 0.5 ? (std::pow(2.0 * t, 2) * ((c2 + 1.0) * 2.0 * t - c2)) / 2.0 : (std::pow(2.0 * t - 2.0, 2) * ((c2 + 1.0) * (t * 2.0 - 2.0) + c2) + 2.0) / 2.0;
    }

    case EaseInElastic: {
      constexpr double c4 = 2.0 * kPi / 3.0;
      if(t <= 0.0) return 0.0;
      if(t >= 1.0) return 1.0;
      return -std::pow(2.0, 10.0 * t - 10.0) * std::sin((t * 10.0 - 10.75) * c4);
    }
    case EaseOutElastic: {
      constexpr double c4 = 2.0 * kPi / 3.0;
      if(t <= 0.0) return 0.0;
      if(t >= 1.0) return 1.0;
      return std::pow(2.0, -10.0 * t) * std::sin((t * 10.0 - 0.75) * c4) + 1.0;
    }
    case EaseInOutElastic: {
      constexpr double c5 = 2.0 * kPi / 4.5;
      if(t <= 0.0) return 0.0;
      if(t >= 1.0) return 1.0;
      return t < 0.5 ? -(std::pow(2.0, 20.0 * t - 10.0) * std::sin((20.0 * t - 11.125) * c5)) / 2.0 : (std::pow(2.0, -20.0 * t + 10.0) * std::sin((20.0 * t - 11.125) * c5)) / 2.0 + 1.0;
    }

    case EaseInBounce: return 1.0 - ease_out_bounce(1.0 - t);
    case EaseOutBounce: return ease_out_bounce(t);
    case EaseInOutBounce: return t < 0.5 ? (1.0 - ease_out_bounce(1.0 - 2.0 * t)) / 2.0 : (1.0 + ease_out_bounce(2.0 * t - 1.0)) / 2.0;

    case Custom: return eval_cubic_bezier(ease_, ease2_, ease3_, ease4_, t);

    case LINEAR:
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
    t        = detail::apply_ease(k0.type, t, k0.ease_, k0.ease2_, k0.ease3_, k0.ease4_);
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
  bool has_key_at(uint32_t frame) const {
    for(auto& k : keys)
      if(k.frame_ == frame) return true;
    return false;
  }
  std::vector<uint32_t> frames() const {
    std::vector<uint32_t> r;
    r.reserve(keys.size());
    for(auto& k : keys) r.push_back(k.frame_);
    return r;
  }
  // 中間点(キーフレーム)を最後の1個以外削除できる。単一値に戻すには残り1個の状態にする
  bool erase_keyframe(uint32_t frame) {
    if(keys.size() <= 1) return false;
    for(auto it = keys.begin(); it != keys.end(); ++it) {
      if(it->frame_ == frame) {
        keys.erase(it);
        return true;
      }
    }
    return false;
  }
  // 指定frameのキーフレームのイージング種別/ベジエハンドルを読み書きする(見つからなければ既定値)
  AniInterpType get_ease_type(uint32_t frame) const {
    for(auto& k : keys)
      if(k.frame_ == frame) return k.type;
    return AniInterpType::LINEAR;
  }
  void set_ease_type(uint32_t frame, AniInterpType t) {
    for(auto& k : keys)
      if(k.frame_ == frame) {
        k.type = t;
        return;
      }
  }
  std::array<float, 4> get_ease_bezier(uint32_t frame) const {
    for(auto& k : keys)
      if(k.frame_ == frame) return {k.ease_, k.ease2_, k.ease3_, k.ease4_};
    return {0.42f, 0.0f, 0.58f, 1.0f};
  }
  void set_ease_bezier(uint32_t frame, std::array<float, 4> v) {
    for(auto& k : keys)
      if(k.frame_ == frame) {
        k.ease_  = v[0];
        k.ease2_ = v[1];
        k.ease3_ = v[2];
        k.ease4_ = v[3];
        return;
      }
  }

  // キーフレームの位置(frame)だけを移動する(値/補間タイプは維持)。移動先に既存キーがあれば上書きする
  bool move_keyframe(uint32_t old_frame, uint32_t new_frame) {
    if(old_frame == new_frame) return true;
    for(size_t i = 0; i < keys.size(); i++) {
      if(keys[i].frame_ != old_frame) continue;
      AnimKeyframe<T> kf = keys[i];
      kf.frame_          = new_frame;
      keys.erase(keys.begin() + i);
      auto it = std::lower_bound(keys.begin(), keys.end(), new_frame, [](const AnimKeyframe<T>& k, uint32_t f) { return k.frame_ < f; });
      if(it != keys.end() && it->frame_ == new_frame)
        *it = kf;
      else
        keys.insert(it, kf);
      return true;
    }
    return false;
  }

  // プロジェクト保存用: キーフレーム列をcutil::Propへ変換する(Entity*等JSON化不可な型では呼ばないこと)
  cutil::Prop save() const {
    cutil::Prop p;
    p.set<int32_t>("count", (int32_t)keys.size());
    for(size_t i = 0; i < keys.size(); i++) {
      cutil::Prop kp;
      kp.set<int32_t>("frame", (int32_t)keys[i].frame_);
      kp.set<int32_t>("interp", (int32_t)keys[i].type);
      kp.set<float>("ease", keys[i].ease_);
      kp.set<float>("ease2", keys[i].ease2_);
      kp.set<float>("ease3", keys[i].ease3_);
      kp.set<float>("ease4", keys[i].ease4_);
      kp.set<T>("value", keys[i].value_);
      p.set_child(("k" + std::to_string(i)).c_str(), kp);
    }
    return p;
  }
  void load(const cutil::Prop& p) {
    keys.clear();
    int32_t n = cutil::get_or<int32_t>(p, "count", 0);
    for(int32_t i = 0; i < n; i++) {
      const std::string key = "k" + std::to_string(i);
      if(!p.contains(key.c_str())) continue;
      const auto& kp = p.get_child(key.c_str());
      AnimKeyframe<T> kf;
      kf.frame_ = (uint32_t)cutil::get_or<int32_t>(kp, "frame", 0);
      kf.type   = (AniInterpType)cutil::get_or<int32_t>(kp, "interp", (int32_t)AniInterpType::LINEAR);
      kf.ease_  = cutil::get_or<float>(kp, "ease", 0.42f);
      kf.ease2_ = cutil::get_or<float>(kp, "ease2", 0.0f);
      kf.ease3_ = cutil::get_or<float>(kp, "ease3", 0.58f);
      kf.ease4_ = cutil::get_or<float>(kp, "ease4", 1.0f);
      kf.value_ = cutil::get_or<T>(kp, "value", T());
      keys.push_back(kf);
    }
    if(keys.empty()) reset();
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

  // UI用: index指定でキーフレームの有無/フレーム一覧/追加/削除/移動を型を意識せず操作する(値はget()/set_value()を使う)
  bool has_key_at(int idx, uint32_t frame) const {
    if(idx < 0 || idx >= (int)props.size()) return false;
    return std::visit([frame](auto&& c) { return c.has_key_at(frame); }, props[idx]);
  }
  bool has_animation(int idx) const {
    if(idx < 0 || idx >= (int)props.size()) return false;
    return std::visit([](auto&& c) { return c.has_animation(); }, props[idx]);
  }
  std::vector<uint32_t> keyframe_frames(int idx) const {
    if(idx < 0 || idx >= (int)props.size()) return {};
    return std::visit([](auto&& c) { return c.frames(); }, props[idx]);
  }
  bool erase_keyframe(int idx, uint32_t frame) {
    if(idx < 0 || idx >= (int)props.size()) return false;
    return std::visit([frame](auto&& c) { return c.erase_keyframe(frame); }, props[idx]);
  }
  bool move_keyframe(int idx, uint32_t old_frame, uint32_t new_frame) {
    if(idx < 0 || idx >= (int)props.size()) return false;
    return std::visit([old_frame, new_frame](auto&& c) { return c.move_keyframe(old_frame, new_frame); }, props[idx]);
  }
  // 現在の補間値をそのままキーフレームとして打つ(AE風「現在値でキーを追加」)
  void add_keyframe_here(int idx, uint32_t frame) {
    if(idx < 0 || idx >= (int)props.size()) return;
    std::visit([frame](auto&& c) { c.add_keyframe(frame, c.get(frame)); }, props[idx]);
  }
  // 明示的な値でキーフレームを打つ(Lua APIなど、has_animation()の状態に関わらず必ずキーフレーム化したい呼び出し元向け)
  template <typename T> bool add_keyframe(int idx, uint32_t frame, T value, AniInterpType type = AniInterpType::LINEAR) {
    if(idx < 0 || idx >= (int)props.size() || !std::holds_alternative<PAniClip<T>>(props[idx])) return false;
    std::get<PAniClip<T>>(props[idx]).add_keyframe(frame, value, type);
    return true;
  }
  // UI用: 指定frameのキーフレームのイージング種別/ベジエハンドルを読み書きする
  AniInterpType get_ease_type(int idx, uint32_t frame) const {
    if(idx < 0 || idx >= (int)props.size()) return AniInterpType::LINEAR;
    return std::visit([frame](auto&& c) { return c.get_ease_type(frame); }, props[idx]);
  }
  void set_ease_type(int idx, uint32_t frame, AniInterpType type) {
    if(idx < 0 || idx >= (int)props.size()) return;
    std::visit([frame, type](auto&& c) { c.set_ease_type(frame, type); }, props[idx]);
  }
  std::array<float, 4> get_ease_bezier(int idx, uint32_t frame) const {
    if(idx < 0 || idx >= (int)props.size()) return {0.42f, 0.0f, 0.58f, 1.0f};
    return std::visit([frame](auto&& c) { return c.get_ease_bezier(frame); }, props[idx]);
  }
  void set_ease_bezier(int idx, uint32_t frame, std::array<float, 4> v) {
    if(idx < 0 || idx >= (int)props.size()) return;
    std::visit([frame, v](auto&& c) { c.set_ease_bezier(frame, v); }, props[idx]);
  }

  bool contains(const std::string& name) const { return index_of(name) >= 0; }
  int index_of(const std::string& name) const {
    for(size_t i = 0; i < props.size(); i++)
      if(std::visit([&name](auto&& arg) { return arg.keyname == name; }, props[i])) return (int)i;
    return -1;
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

  // プロジェクト保存用: 全プロパティのキーフレーム列をcutil::Propへ変換する/そこから復元する(名前一致でマージ、propsは事前にadd_props()等で構築済みであること)
  cutil::Prop save() const;
  void load_keys(const cutil::Prop& saved);
};

} // namespace mu
