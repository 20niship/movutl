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
};

template <typename T> struct PropAnimClip {
public:
  std::vector<AnimKeyframe<T>> keys;
  std::string keyname;

  PropAnimClip() { reset(); }
  void reset() {
    keys.clear();
    keys.push_back({T(), 0, 0, 0, AniInterpType::LINEAR});
  }
  T get(uint32_t frame);
  bool add_keyframe(uint32_t frame, T value, AniInterpType t = AniInterpType::LINEAR);
  bool has_animation() const { return keys.size() > 1; }
  void clear() { keys.clear(); }
};

struct AnimProps {
public:
  // clang-format off
  using Types = std::variant< \
        PropAnimClip<int>,  \
        PropAnimClip<float>,  \
        PropAnimClip<std::string>, \
        PropAnimClip<bool>, \
        PropAnimClip<Vec2>, \
        PropAnimClip<Vec3>, \
        PropAnimClip<Vec3b>,  \
        PropAnimClip<Entity*>\
    >;
  // clang-format on

  std::vector<Types> props;
  Props get(uint32_t frame);
  size_t size() const { return props.size(); }

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
};

} // namespace mu
