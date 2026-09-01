#pragma once
// mu::Vec2/Vec3/Vec4/Vec4b, std::string, Entity*用のcutil::PropInfoOf<T>特殊化(movutl側の型は変更しない)。
#include <cutil/prop.hpp>
#include <movutl/core/vector.hpp>
#include <string>

namespace mu {
class Entity;
}

namespace cutil {

namespace detail {
template <typename VecT, size_t N> const PropInfo* make_mu_vec_propinfo(const char* id) {
  static const PropInfo info = [id] {
    PropInfo p;
    p.set_id(id);
    p.klass   = PropClass::Trivial;
    p.size    = sizeof(VecT);
    p.align   = alignof(VecT);
    p.to_json = [](const void* obj) -> std::string { return detail::json_array_of_floats(reinterpret_cast<const float*>(reinterpret_cast<const VecT*>(obj)->value), N); };
    if constexpr(std::is_same_v<decltype(VecT::value[0]), float>) {
      p.from_json = [](void* obj, const std::string& text) -> bool {
        detail::json_floats_from_array(text, reinterpret_cast<VecT*>(obj)->value, N);
        return true;
      };
    }
    return p;
  }();
  return &info;
}
} // namespace detail

template <> struct PropInfoOf<mu::Vec2> {
  static const PropInfo* get() { return detail::make_mu_vec_propinfo<mu::Vec2, 2>("mu::Vec2"); }
};
template <> struct PropInfoOf<mu::Vec3> {
  static const PropInfo* get() { return detail::make_mu_vec_propinfo<mu::Vec3, 3>("mu::Vec3"); }
};
template <> struct PropInfoOf<mu::Vec4> {
  static const PropInfo* get() { return detail::make_mu_vec_propinfo<mu::Vec4, 4>("mu::Vec4"); }
};

// Vec4b(unsigned char x4)はfloat配列ではないのでJSON変換は素朴なint配列で書く。
template <> struct PropInfoOf<mu::Vec4b> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("mu::Vec4b");
      p.klass   = PropClass::Trivial;
      p.size    = sizeof(mu::Vec4b);
      p.align   = alignof(mu::Vec4b);
      p.to_json = [](const void* obj) -> std::string {
        const auto* v = reinterpret_cast<const mu::Vec4b*>(obj);
        return "[" + std::to_string(v->value[0]) + "," + std::to_string(v->value[1]) + "," + std::to_string(v->value[2]) + "," + std::to_string(v->value[3]) + "]";
      };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        auto* v = reinterpret_cast<mu::Vec4b*>(obj);
        float tmp[4];
        detail::json_floats_from_array(text, tmp, 4);
        for(int i = 0; i < 4; i++) v->value[i] = static_cast<unsigned char>(tmp[i]);
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// std::string: 可変長なのでIndirectとして扱う(cutil::Strへは寄せず、movutl側の型のまま)。
template <> struct PropInfoOf<std::string> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("std::string");
      p.klass        = PropClass::Indirect;
      p.size         = sizeof(std::string);
      p.align        = alignof(std::string);
      p.copy_ctor    = [](void* dst, const void* src) { new(dst) std::string(*reinterpret_cast<const std::string*>(src)); };
      p.dtor         = [](void* obj) { reinterpret_cast<std::string*>(obj)->~basic_string(); };
      p.default_ctor = [](void* obj) { new(obj) std::string(); };
      p.to_json      = [](const void* obj) -> std::string { return detail::json_quote(*reinterpret_cast<const std::string*>(obj)); };
      p.from_json    = [](void* obj, const std::string& text) -> bool {
        new(obj) std::string(detail::json_unquote(detail::json_trim(text)));
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// Entity*: AnimProps<Entity*>用の生ポインタ格納(memcpyのみ、JSON化はしない)。
template <> struct PropInfoOf<mu::Entity*> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("mu::Entity*");
      p.klass = PropClass::Trivial;
      p.size  = sizeof(mu::Entity*);
      p.align = alignof(mu::Entity*);
      return p;
    }();
    return &info;
  }
};

// uint32_t: cutilはint32_t/uint8_tのみ組み込みなので別途登録する(Entity::group_guid用)。
template <> struct PropInfoOf<uint32_t> {
  static const PropInfo* get() {
    static const PropInfo info = [] {
      PropInfo p;
      p.set_id("uint32_t");
      p.klass     = PropClass::Trivial;
      p.size      = sizeof(uint32_t);
      p.align     = alignof(uint32_t);
      p.to_json   = [](const void* obj) -> std::string { return std::to_string(*reinterpret_cast<const uint32_t*>(obj)); };
      p.from_json = [](void* obj, const std::string& text) -> bool {
        *reinterpret_cast<uint32_t*>(obj) = static_cast<uint32_t>(std::stoul(detail::json_trim(text)));
        return true;
      };
      return p;
    }();
    return &info;
  }
};

// 型不一致・未設定なら既定値を返す(旧Props::get_orの代替、cutil::Prop::get<T>は型不一致で例外を投げるため)。
template <typename T> T get_or(const cutil::Prop& p, const char* name, const T& def) {
  if(!p.contains(name)) return def;
  try {
    return p.get<T>(name);
  } catch(...) {
    return def;
  }
}

} // namespace cutil
