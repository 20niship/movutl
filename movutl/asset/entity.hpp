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
protected:
  InputPluginTable* in_plg_ = nullptr;

public:
  char name[MAX_DISPNAME];
  uint64_t guid_ = 0;
  TrackObject trk;

  virtual constexpr EntityType getType() const = 0;

  Ref<Entity> CreateEntity(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);

  Ref<Composition> get_comp() const;
  virtual bool render(Composition* cmp) = 0;

  bool visible(int frame) const { return trk.visible(frame); }
};

} // namespace mu
