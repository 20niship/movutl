#include <movutl/core/anim.hpp>
#include <movutl/core/props.hpp>

namespace mu {
void AnimProps::add_props(const Props& props) {
  for(auto [key, value] : props.values) {
    if(this->contains(key)) this->erase(key);

    if(std::holds_alternative<float>(value))
      this->props.push_back(PAniClip<float>(key, std::get<float>(value)));

    else if(std::holds_alternative<int>(value))
      this->props.push_back(PAniClip<int>(key, std::get<int>(value)));

    else if(std::holds_alternative<std::string>(value))
      this->props.push_back(PAniClip<std::string>(key, std::get<std::string>(value)));

    else if(std::holds_alternative<bool>(value))
      this->props.push_back(PAniClip<bool>(key, std::get<bool>(value)));

    else if(std::holds_alternative<Vec2>(value))
      this->props.push_back(PAniClip<Vec2>(key, std::get<Vec2>(value)));

    else if(std::holds_alternative<Vec3>(value))
      this->props.push_back(PAniClip<Vec3>(key, std::get<Vec3>(value)));

    else if(std::holds_alternative<Vec4>(value))
      this->props.push_back(PAniClip<Vec4>(key, std::get<Vec4>(value)));

    else if(std::holds_alternative<Vec4b>(value))
      this->props.push_back(PAniClip<Vec4b>(key, std::get<Vec4b>(value)));
  }
}

struct PropsSetVisitor {
  PropsSetVisitor(Props& props) : props(props) {}
  Props& props;
  template <typename T> void operator()(const PAniClip<T>& clip) { props[clip.keyname] = clip.get(0); }
};

Props AnimProps::get(uint32_t frame) {
  Props p;
  PropsSetVisitor visitor(p);
  for(auto& prop : props) std::visit(visitor, prop);
  return p;
}

} // namespace mu
