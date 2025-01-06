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
  EntityType_Polygon,
  EntityType_Group,
  EntityType_Scene,
  EntityType_SceneAudio,
  EntityType_LayerCopy,
  EntityType_Particle,
  EntityType_Custom,
  EntityType_3DModel,
  EntityType_Camera,
  EntityType_Effect,
};

class Entity {
public:
  std::string name_;
  uint32_t guid_;
  virtual constexpr EntityType getType() const = 0;
};

} // namespace mu
