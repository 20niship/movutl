#pragma once

#include <movutl/asset/track.hpp>
#include <movutl/core/defines.hpp>
#include <string>

namespace mu {

struct InputPluginTable;
class Composition;

enum EntityType {
  EntityType_Movie = 1,
  EntityType_Audio = 1 << 1,
  EntityType_Image = 1 << 2,
  EntityType_3DText = 1 << 3,
  EntityType_Primitive = 1 << 4,
  EntityType_Framebuffer = 1 << 5,
  EntityType_Polygon = 1 << 6,
  EntityType_Group = 1 << 7,
  EntityType_Scene = 1 << 8,
  EntityType_SceneAudio = 1 << 9,
  EntityType_LayerCopy = 1 << 10,
  EntityType_Particle = 1 << 11,
  EntityType_Custom = 1 << 12,
  EntityType_3DModel = 1 << 13,
  EntityType_Camera = 1 << 14,
  EntityType_Effect = 1 << 15,
};
MOVUTL_DEFINE_ENUM_ATTR_BITFLAGS(EntityType);

// プラグインがそのファイルを開いた時のインスタンスを返すときのポインタ
typedef void* InputHandle;

class Entity {
protected:
  InputPluginTable* in_plg_ = nullptr;
  InputHandle in_handle_ = nullptr;

public:
  char name[MAX_DISPNAME];
  uint64_t guid_ = 0;
  TrackObject trk;

  virtual constexpr EntityType getType() const = 0;

  Ref<Entity> CreateEntity(const char* name, EntityType type);
  static Ref<Entity> Find(const char* name);

  Composition* get_comp() const;
  virtual bool render(Composition* cmp) = 0;

  bool visible(int frame) const { return trk.visible(frame); }
  virtual ~Entity();
};

} // namespace mu
