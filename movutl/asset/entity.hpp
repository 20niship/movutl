#pragma once

#include <movutl/asset/track.hpp>
#include <string>

namespace mu {

struct InputPluginTable;
class Composition;

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
private:
  InputPluginTable* in_plg_ = nullptr;

public:
  char name[MAX_DISPNAME];
  uint64_t guid_ = 0;
  TrackObject trk_;

  virtual constexpr EntityType getType() const = 0;
  int width = 0;
  int height = 0;

  static Ref<Entity> Create(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);
  static Ref<Entity> LoadFile(const char* name, const char* path);

  Ref<Composition> get_comp() const;
};

} // namespace mu
