#pragma once
#include <movutl/core/vector.hpp>

namespace mu {
struct Range {
  float min, max;
  Range() { clear(); }
  Range(float min_, float max_) {
    min = min_;
    max = max_;
  }
  float length() const { return max - min; }
  float center() const { return (max + min) / 2.0f; }
  void clear() {
    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::lowest();
  }
  bool contains(float t) const { return min <= t && t <= max; }
  bool contains(const Range& t) const { return !(t.max <= min || t.min >= max); }
  void merge(const Range& t) {
    min = std::min(min, t.min);
    max = std::max(max, t.max);
  }
  void expand(const float& t) {
    min = std::min(min, t);
    max = std::max(max, t);
  }
  Range scale(const float t) const { return Range(center() - length() * t / 2.0f, center() + length() * t / 2.0f); }
  Range margin(const float t) const { return Range(min - t, max + t); }
  Range shift(float t) const { return Range(min + t, max + t); }

  bool valid() const { return max >= min; }
  bool operator==(const Range o) const { return o.min == min && o.max == max; }
  [[nodiscard]] std::string str() const { return "Range(" + std::to_string(min) + ", " + std::to_string(max) + ")"; }
  static Range Inf() { return Range(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max()); }
};

struct Rect3D {
  Range x, y, z;
  Rect3D() = default;
  Rect3D(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
    x.min = xmin;
    x.max = xmax;
    y.min = ymin;
    y.max = ymax;
    z.min = zmin;
    z.max = zmax;
  }

  Rect3D(const Vec2& range_x, const Vec2& range_y, const Vec2& range_z) {
    x.min = range_x[0];
    y.min = range_y[0];
    z.min = range_z[0];
    x.max = range_x[0];
    y.max = range_y[1];
    z.max = range_z[2];
  }

  Rect3D(float min, float max) {
    x.min = y.min = z.min = min;
    x.max = y.max = z.max = max;
  }

  Rect3D(const Range& range_x, const Range& range_y, const Range& range_z) {
    x = range_x;
    y = range_y;
    z = range_z;
  }

  Rect3D(const Vec3& pos, const Vec3& size) {
    x.min = pos[0];
    y.min = pos[1];
    z.min = pos[2];
    x.max = pos[0] - size[0];
    y.max = pos[1] - size[1];
    z.max = pos[2] - size[2];
  }
  auto w() const { return x.length(); } /// width  ( x length  )
  auto d() const { return y.length(); } /// depth  ( y length  )
  auto h() const { return z.length(); } /// height ( z lngth   )
  Vec3 pos() const { return {x.min, y.min, z.min}; }
  Vec3 center() const { return {x.center(), y.center(), z.center()}; }
  Vec3 size() const { return {x.length(), y.length(), z.length()}; }
  bool contains(const Rect3D& other) const { return x.contains(other.x) && y.contains(other.y) && z.contains(other.z); }
  bool contains(const Vec3& p) const { return x.contains(p[0]) && y.contains(p[1]) && z.contains(p[2]); }
  bool contains(const float x_, float y_, float z_) const { return x.contains(x_) && y.contains(y_) && z.contains(z_); }
  float area() const { return x.length() * y.length() * z.length(); }
  void clear() {
    x.clear();
    y.clear();
    z.clear();
  }
  void expand(float x_, float y_, float z_) { expand(Vec3(x_, y_, z_)); }
  void expand(const Vec3& p) {
    x.expand(p[0]);
    y.expand(p[1]);
    z.expand(p[2]);
  }
  bool valid() const { return x.valid() && y.valid() && z.valid(); }
  Rect3D scale(float t) const { return Rect3D(x.scale(t), y.scale(t), z.scale(t)); }
  Rect3D scale(const Vec3 t) { return Rect3D(x.scale(t[0]), y.scale(t[1]), z.scale(t[2])); }
  Rect3D margin(float t) const { return Rect3D(x.margin(t), y.margin(t), z.margin(t)); }

  void merge(const Rect3D& other) {
    x.merge(other.x);
    y.merge(other.y);
    z.merge(other.z);
  }
  static Rect3D Inf() { return Rect3D(Range::Inf(), Range::Inf(), Range::Inf()); }
  [[nodiscard]] std::string str() const { return "Rect3D( [x,y,z] = " + x.str() + ", " + y.str() + ", " + z.str() + ")"; }
};

struct Rect {
  Range x, y;
  Rect() { x.min = y.min = x.max = y.max = 0; }
  Rect(const Range& xr, const Range& yr) {
    x = xr;
    y = yr;
  }
  Rect(float xmin, float xmax, float ymin, float ymax) {
    x.min = xmin;
    x.max = xmax;
    y.min = ymin;
    y.max = ymax;
  }
  Rect(const _Vec<float, 2>& pos, const _Vec<float, 2>& size) {
    x.min = pos[0];
    x.max = pos[0] + size[0];
    y.min = pos[1];
    y.max = pos[1] + size[1];
  }
  Vec2 center() const { return {x.center(), y.center()}; }
  Vec2 size() const { return {x.length(), y.length()}; }
  Vec2 pos() const { return {x.min, y.min}; }
  float top() const { return y.min; }
  float bottom() const { return y.max; }
  float left() const { return x.min; }
  float right() const { return x.max; }

  void top(float t) { y.min = t; }
  void bottom(float t) { y.max = t; }
  void left(float t) { x.min = t; }
  float w() { return x.length(); }
  float h() { return y.length(); }
  void right(float t) { x.max = t; }

  bool contains(const Rect& other) const { return x.contains(other.x) && y.contains(other.y); }
  bool contains(const _Vec<float, 2>& p) const { return contains(p[0], p[1]); }
  bool contains(const float x_, const float y_) const { return x.contains(x_) && y.contains(y_); }
  float area() const { return x.length() * y.length(); }
  void expand(const float x_, const float y_) { expand({x_, y_}); }
  void expand(const Vec2& p) {
    x.expand(p[0]);
    y.expand(p[1]);
  }
  bool valid() const { return x.valid() && y.valid(); }
  void scale(float t) {
    x.scale(t);
    y.scale(t);
  }
  void merge(const Rect& other) {
    x.merge(other.x);
    y.merge(other.y);
  }
  void clear() {
    x.clear();
    y.clear();
  }
  static Rect Inf() { return Rect(Range::Inf(), Range::Inf()); }
};

/*template <typename float> inline std::ostream& operator<<(std::ostream& os, const Range& r) {*/
/*  return os << "Range(" << r.min << ", " << r.max << ")";*/
/*}*/
/*inline std::ostream& operator<<(std::ostream& os, const Rect3D& r) {*/
/*  return os << "Rect(x, y, z) = [ " << r.x << ", " << r.y << ", " << r.z << " ] ";*/
/*}*/
/*inline std::ostream& operator<<(std::ostream& os, const Rect& r) {*/
/*  return os << "Rect(x, y) = [ " << r.x << ", " << r.y << " ]  ";*/
/*}*/
/**/
} // namespace mu
