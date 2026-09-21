#include <algorithm>
#include <cmath>
#include <movutl/asset/camera.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

Ref<Camera3D> Camera3D::Create(const char* name) {
  auto c  = cutil::make_ref<Camera3D>();
  c->name = name;
  Project::Get()->entities.push_back(c);
  c->guid_ = Project::Get()->entities.size();
  return c;
}

GroupXform Camera3D::view_xform() const {
  const float dist = -pos_[2]; // Z=0面までの距離。既定(-1024)で1024
  const float s    = dist >= 1.0f ? -kCameraDefaultZ / dist : 1.0f;
  const double rad = -rotation_ * M_PI / 180.0;
  const double c = std::cos(rad), sn = std::sin(rad);
  const double px = -pos_[0] * s, py = -pos_[1] * s;
  GroupXform x;
  x.scale    = 100.0f * s;
  x.rotation = -rotation_;
  x.pos      = Vec3((float)(px * c - py * sn), (float)(px * sn + py * c), 0.0f);
  return x;
}

} // namespace mu
