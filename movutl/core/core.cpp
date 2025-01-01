#include <cmath>
#include <movutl/core/core.hpp>

namespace mu::core {

using namespace mu::core;

Mat4x4 rotation_mat_4x4(const Vec3& pry) {
  Mat4x4 p, r, y;
  // clang-format off
  p << 1,0,0,0,
    0,std::cos(pry[0]) , -std::sin(pry[0]),0, 
    0,std::sin(pry[0]) , std::cos(pry[0]),0, 
    0,0,0,1;
  p(0,0) = 1;

  r << std::cos(pry[1]), 0,-std::sin(pry[1]),0,
    0,1,0,0,
    std::sin(pry[1]),0,std::cos(pry[1]),0,
    0,0,0,1;
  r(0,0) = std::cos(pry[1]);

  y << std::cos(pry[2]) , -std::sin(pry[2]),0,0,
    std::sin(pry[2]), std::cos(pry[2]), 0,0,
    0,0,1,0,
    0,0,0,1;
  y(0,0) = std::cos(pry[2]);


  // clang-format on
  return p * r * y;
}

Mat4x4 scale_mat(const Vec3& scale) {
  Mat4x4 s;
  // clang-format off
  s << scale[0],0,0,0,
       0,scale[0],0,0,
       0,0,scale[0],0,
       0,0,0,1;
  // clang-format on
  return s;
}

Mat4x4 translation_mat(const Vec3& pos) {
  Mat4x4 s;
  // clang-format off
  s << 1,0,0,pos[0],
       0,1,0,pos[1],
       0,0,1,pos[2],
       0,0,0,1;
  // clang-format on
  return s;
}

Mat4x4 transform_mat(const Vec3& pos, const Vec3& pry, const Vec3& scale) {
  return rotation_mat_4x4(pry) * scale_mat(scale) * translation_mat(pos);
}

std::array<Vec3, 2> get_vert_vec(const Vec3& x) {
  Vec3 e1, e2;
  if(x[0] >= x[1] && x[0] >= x[2]) { // X軸に近い直線
    e1 = {x[1], x[0], x[2]};
    e2 = {x[2], x[1], x[0]};
  } else if(x[1] >= x[0] && x[1] >= x[2]) {
    e1 = {x[1], x[0], x[2]};
    e2 = {x[0], x[2], x[1]};
  } else {
    e1 = {x[2], x[1], x[0]};
    e2 = {x[0], x[2], x[1]};
  }

  e1 = x.cross(e1);
  e1 = e1.normalize();

  e2 = x.cross(e1);
  e2 = e2.normalize();

  return {e1, e2};
}


} // namespace mu::core
