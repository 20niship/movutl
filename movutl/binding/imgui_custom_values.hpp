#pragma once
#include <LuaIntf/LuaIntf.h>
#include <imgui.h>
#include <movutl/binding/binding.hpp>
#include <movutl/core/vector.hpp>

namespace mu::detail {
using namespace LuaIntf;
template <typename T, int N> inline void bind_custom_vectors(CppBindModule<LuaBinding>& L, const char* name) {
  auto cls = L.beginClass<_Vec<T, N>>(name).addConstructor(LUA_ARGS());
  if constexpr(N == 2)
    cls = cls.addConstructor(LUA_ARGS(T, T));
  else if constexpr(N == 3)
    cls = cls.addConstructor(LUA_ARGS(T, T, T));
  else if constexpr(N == 4)
    cls = cls.addConstructor(LUA_ARGS(T, T, T, T));

  using V = _Vec<T, N>;
  cls.addProperty("x", static_cast<T (V::*)() const>(&V::x), static_cast<void (V::*)(T)>(&V::x));
  cls.addProperty("y", static_cast<T (V::*)() const>(&V::y), static_cast<void (V::*)(T)>(&V::y));

  if constexpr(N >= 3) cls.addProperty("z", static_cast<T (V::*)() const>(&V::z), static_cast<void (V::*)(T)>(&V::z));
  if constexpr(N == 3) cls = cls.addFunction("cross", &V::cross);

  cls.addFunction("normalize", &V::normalize)
    .addFunction("norm_sq", &V::norm_sq)
    .addFunction("norm", &V::norm)
    .addFunction("all", &V::all)
    .addFunction("str", &V::str)
    .addFunction("avg", &V::avg)
    .addFunction("min", &V::min)
    .addFunction("max", &V::max)
    .addFunction("sum", &V::sum)
    .addFunction("dot", &V::dot)
    .endClass();
}

template <typename T> inline void bind_imgui_vectors(CppBindModule<LuaBinding>& L, const char* name) {
  auto cls = L.beginClass<T>(name).addConstructor(LUA_ARGS());

  if constexpr(std::is_same_v<T, ImVec2>) {
    cls = cls
            .addConstructor(LUA_ARGS())             //
            .addConstructor(LUA_ARGS(float, float)) //
            .addVariable("x", &T::x)                //
            .addVariable("y", &T::y);
  }

  if constexpr(std::is_same_v<T, ImVec4>) {
    cls = cls
            .addConstructor(LUA_ARGS())                           //
            .addConstructor(LUA_ARGS(float, float, float, float)) //
            .addVariable("x", &T::x)                              //
            .addVariable("y", &T::y)                              //
            .addVariable("z", &T::z)                              //
            .addVariable("w", &T::w);
  }
  cls.endClass();
}

inline void binding_custom_vectors(lua_State* L) {
  auto imgui = LuaBinding(L).beginModule("imgui");
  bind_imgui_vectors<ImVec2>(imgui, "ImVec2");
  bind_imgui_vectors<ImVec4>(imgui, "ImVec4");
  imgui.endModule();

  auto movutl = LuaBinding(L).beginModule("movutl");
  bind_custom_vectors<float, 2>(movutl, "Vec2");
  bind_custom_vectors<float, 3>(movutl, "Vec3");
  bind_custom_vectors<float, 4>(movutl, "Vec4");

  movutl.endModule();
}
} // namespace mu::detail
