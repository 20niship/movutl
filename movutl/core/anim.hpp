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
  AniInterpType type;
};

template <typename T> struct PropAnimationClip {
  std::vector<AnimKeyframe<T>> keys;
  std::string keyname;

  T get(uint32_t frame);
  bool add_keyframe(uint32_t frame, T value, AniInterpType t = AniInterpType::LINEAR);
};


} // namespace mu
