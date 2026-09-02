#include <movutl/core/anim.hpp>

namespace mu {
void AnimProps::add_props(const cutil::Prop& defaults) {
  for(const auto& f : defaults.fields()) {
    if(this->contains(f.name)) this->erase(f.name);

    if(f.type == cutil::prop_info_of<float>())
      this->props.push_back(PAniClip<float>(f.name, defaults.get<float>(f.name)));
    else if(f.type == cutil::prop_info_of<int32_t>())
      this->props.push_back(PAniClip<int>(f.name, defaults.get<int32_t>(f.name)));
    else if(f.type == cutil::prop_info_of<std::string>())
      this->props.push_back(PAniClip<std::string>(f.name, defaults.get<std::string>(f.name)));
    else if(f.type == cutil::prop_info_of<bool>())
      this->props.push_back(PAniClip<bool>(f.name, defaults.get<bool>(f.name)));
    else if(f.type == cutil::prop_info_of<Vec2>())
      this->props.push_back(PAniClip<Vec2>(f.name, defaults.get<Vec2>(f.name)));
    else if(f.type == cutil::prop_info_of<Vec3>())
      this->props.push_back(PAniClip<Vec3>(f.name, defaults.get<Vec3>(f.name)));
    else if(f.type == cutil::prop_info_of<Vec4>())
      this->props.push_back(PAniClip<Vec4>(f.name, defaults.get<Vec4>(f.name)));
    else if(f.type == cutil::prop_info_of<Vec4b>())
      this->props.push_back(PAniClip<Vec4b>(f.name, defaults.get<Vec4b>(f.name)));
  }
}

struct PropsSetVisitor {
  PropsSetVisitor(cutil::Prop& props) : props(props) {}
  cutil::Prop& props;
  template <typename T> void operator()(const PAniClip<T>& clip) { props.set<T>(clip.keyname.c_str(), clip.get(0)); }
};

cutil::Prop AnimProps::get(uint32_t frame) const {
  MU_UNUSED(frame);
  cutil::Prop p;
  PropsSetVisitor visitor(p);
  for(auto& prop : props) std::visit(visitor, prop);
  return p;
}

} // namespace mu
