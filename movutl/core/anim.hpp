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
      kf.ease_  = cutil::get_or<float>(kp, "ease", 0.0f);
      kf.ease2_ = cutil::get_or<float>(kp, "ease2", 0.0f);
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
