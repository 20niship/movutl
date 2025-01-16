#pragma once
#include <imgui.h>
#include <movutl/core/vector.hpp>
#include <string>

namespace ImGui {

using namespace mu;
using S = const std::string&;

// clang-format off
inline void Text_(S text) { Text("%s", text.c_str()); }
inline void TextColored_(S text, const ImVec4& color) { TextColored(color, "%s", text.c_str()); }
inline void TextDisabled_(S text) { TextDisabled("%s", text.c_str()); }
inline void TextWrapped_(S text) { TextWrapped("%s", text.c_str()); }
inline void LabelText_(S label, S text) { LabelText(label.c_str(), "%s", text.c_str()); }
inline void DebugLog_(S text) { DebugLog("%s", text.c_str()); }
inline void BulletText_(S text) { BulletText("%s", text.c_str()); }

inline Vec2 InputFloat2_(S label, Vec2 v) { InputFloat2(label.c_str(), &v[0]); return v; }
inline Vec3 InputFloat3_(S label, Vec3 v) { InputFloat3(label.c_str(), &v[0]); return v; }
inline Vec4 InputFloat4_(S label, Vec4 v) { InputFloat4(label.c_str(), &v[0]); return v; }

inline Vec2d InputInt2_(S label, Vec2d v) { int v2[2] = {v[0], v[1]}; InputInt2(label.c_str(), v2); return Vec2d(v2[0], v2[1]); }
inline Vec3d InputInt3_(S label, Vec3d v) { int v3[3] = {v[0], v[1], v[2]}; InputInt3(label.c_str(), v3); return Vec3d(v3[0], v3[1], v3[2]); }
inline Vec4d InputInt4_(S label, Vec4d v) { int v4[4] = {v[0], v[1], v[2], v[3]}; InputInt4(label.c_str(), v4); return Vec4d(v4[0], v4[1], v4[2], v4[3]); }

inline Vec2 DragFloat2_(S label, Vec2 v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { DragFloat2(label.c_str(), &v[0], speed, min, max); return v; }
inline Vec3 DragFloat3_(S label, Vec3 v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { DragFloat3(label.c_str(), &v[0], speed, min, max); return v; }
inline Vec4 DragFloat4_(S label, Vec4 v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { DragFloat4(label.c_str(), &v[0], speed, min, max); return v; }

inline Vec2d DragInt2_(S label, Vec2d v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { int v2[2] = {v[0], v[1]}; DragInt2(label.c_str(), v2, speed, min, max); return Vec2d(v2[0], v2[1]); }
inline Vec3d DragInt3_(S label, Vec3d v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { int v3[3] = {v[0], v[1], v[2]}; DragInt3(label.c_str(), v3, speed, min, max); return Vec3d(v3[0], v3[1], v3[2]); }
inline Vec4d DragInt4_(S label, Vec4d v, float speed = 1.0f, float min = 0.0f, float max = 0.0f) { int v4[4] = {v[0], v[1], v[2], v[3]}; DragInt4(label.c_str(), v4, speed, min, max); return Vec4d(v4[0], v4[1], v4[2], v4[3]); }

inline Vec2 SliderFloat2_(S label, Vec2 v, float min, float max) { SliderFloat2(label.c_str(), &v[0], min, max); return v; }
inline Vec3 SliderFloat3_(S label, Vec3 v, float min, float max) { SliderFloat3(label.c_str(), &v[0], min, max); return v; }
inline Vec4 SliderFloat4_(S label, Vec4 v, float min, float max) { SliderFloat4(label.c_str(), &v[0], min, max); return v; }

inline Vec2d SliderInt2_(S label, Vec2d v, int min, int max) { int v2[2] = {v[0], v[1]}; SliderInt2(label.c_str(), v2, min, max); return Vec2d(v2[0], v2[1]); }
inline Vec3d SliderInt3_(S label, Vec3d v, int min, int max) { int v3[3] = {v[0], v[1], v[2]}; SliderInt3(label.c_str(), v3, min, max); return Vec3d(v3[0], v3[1], v3[2]); }
inline Vec4d SliderInt4_(S label, Vec4d v, int min, int max) { int v4[4] = {v[0], v[1], v[2], v[3]}; SliderInt4(label.c_str(), v4, min, max); return Vec4d(v4[0], v4[1], v4[2], v4[3]); }

inline void ColorEdit3_(S label, Vec3& color) { ColorEdit3(label.c_str(), &color[0]); }
inline void ColorEdit4_(S label, Vec4& color) { ColorEdit4(label.c_str(), &color[0]); }
inline void ColorPicker3_(S label, Vec3& color) { ColorPicker3(label.c_str(), &color[0]); }
inline void ColorPicker4_(S label, Vec4& color) { ColorPicker4(label.c_str(), &color[0]); }

inline bool TreeNodeEx_(S label, ImGuiTreeNodeFlags flags = 0) { return TreeNodeEx(label.c_str(), flags); }

} // namespace ImGui
