#pragma once
#include <string>

namespace mu {

enum EntityType {
  EntityType_Movie,
  EntityType_Audio,
  EntityType_Image,
  EntityType_3DText,
  EntityType_Primitive,
  EntityType_Framebuffer,
  EntityType_Scene,
  EntityType_LayerCopy,
  EntityType_Particle,
  EntityType_Custom,
  EntityType_3DModel,
};

class Entity {
public:
  std::string name_;
  int guid_;
  virtual constexpr EntityType getType() const = 0;
};

} // namespace mu
