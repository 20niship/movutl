#include <movutl/asset/mesh.hpp>

namespace mu {

Vec3 Mesh::get_center_of_geom() const {
  if(vertices.size() == 0) return {0, 0, 0};
  Vec3 p(0, 0, 0);
  const Vertex* end = vertices.end();
  for(auto* ptr = vertices.begin(); ptr < end; ptr++) {
    p[0] += ptr->pos[0];
    p[1] += ptr->pos[1];
    p[2] += ptr->pos[2];
  }
  return p / vertices.size();
}
Rect3D Mesh::get_bbox() const {
  Rect3D r;
  const Vertex* end = vertices.end();
  for(auto* ptr = vertices.begin(); ptr <= end; ptr++) {
    r.expand(ptr->pos[0], ptr->pos[1], ptr->pos[2]);
  }
  return r;
}
void Mesh::move_geom(const Vec3& dp) {
  const Vertex* end = vertices.end();
  for(auto* ptr = vertices.begin(); ptr <= end; ptr++) {
    ptr->pos[0] += dp[0];
    ptr->pos[1] += dp[1];
    ptr->pos[2] += dp[2];
  }
}
void Mesh::scale_geom(const Vec3& dp) {
  const Vertex* end = vertices.end();
  for(auto* ptr = vertices.begin(); ptr <= end; ptr++) {
    ptr->pos[0] *= dp[0];
    ptr->pos[1] *= dp[1];
    ptr->pos[2] *= dp[2];
  }
}
void Mesh::rot_geom(const Vec3& rot) {
  const float sin = std::sin(rot[2]);
  const float cos = std::cos(rot[2]);
  const auto* end = vertices.end();
  for(auto* ptr = vertices.begin(); ptr <= end; ptr++) {
    const auto x = ptr->pos[0];
    const auto y = ptr->pos[1];
    ptr->pos[0] = cos * x - sin * y;
    ptr->pos[1] = sin * x + cos * y;
  }
}

} // namespace mu
