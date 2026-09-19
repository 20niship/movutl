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
  PropsSetVisitor(cutil::Prop& props, uint32_t frame) : props(props), frame(frame) {}
  cutil::Prop& props;
  uint32_t frame;
  template <typename T> void operator()(const PAniClip<T>& clip) { props.set<T>(clip.keyname.c_str(), clip.get(frame)); }
};

cutil::Prop AnimProps::get(uint32_t frame) const {
  MU_UNUSED(frame);
  cutil::Prop p;
  PropsSetVisitor visitor(p, frame);
  for(auto& prop : props) std::visit(visitor, prop);
  return p;
}

namespace {
// Entity*はJSON化不可(to_json未登録)のため保存対象から除外する
struct PropsSaveVisitor {
  cutil::Prop& out;
  int& idx;
  template <typename T> void operator()(const PAniClip<T>& clip) {
    if constexpr(std::is_same_v<T, Entity*>) {
      return;
    } else {
      cutil::Prop cp = clip.save();
      cp.set<std::string>("name", clip.keyname);
      out.set_child(("p" + std::to_string(idx)).c_str(), cp);
      idx++;
    }
  }
};

struct PropsLoadVisitor {
  const cutil::Prop& node;
  template <typename T> void operator()(PAniClip<T>& clip) {
    if constexpr(std::is_same_v<T, Entity*>) {
      return;
    } else {
      clip.load(node);
    }
  }
};
} // namespace

cutil::Prop AnimProps::save() const {
  cutil::Prop out;
  int idx = 0;
  PropsSaveVisitor visitor{out, idx};
  for(auto& prop : props) std::visit(visitor, prop);
  out.set<int32_t>("count", idx);
  return out;
}

void AnimProps::load_keys(const cutil::Prop& saved) {
  int32_t count = cutil::get_or<int32_t>(saved, "count", 0);
  for(int32_t i = 0; i < count; i++) {
    const std::string key = "p" + std::to_string(i);
    if(!saved.contains(key.c_str())) continue;
    const auto& node       = saved.get_child(key.c_str());
    const std::string name = cutil::get_or<std::string>(node, "name", "");
    for(auto& prop : props) {
      bool match = std::visit([&name](auto&& c) { return c.keyname == name; }, prop);
      if(!match) continue;
      PropsLoadVisitor visitor{node};
      std::visit(visitor, prop);
      break;
    }
  }
}

} // namespace mu
