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

  T get(uint32_t frame);
  bool add_keyframe(uint32_t frame, T value, AniInterpType t = AniInterpType::LINEAR);
};

struct AnimProps {
public:
  using Types = std::variant<PropAnimClip<int>, PropAnimClip<float>, PropAnimClip<std::string>, PropAnimClip<bool>, PropAnimClip<Vec2>, PropAnimClip<Vec3>, PropAnimClip<Vec3b>, PropAnimClip<Entity*>>;

  std::vector<Types> props;
  Props get(uint32_t frame);
};

} // namespace mu
