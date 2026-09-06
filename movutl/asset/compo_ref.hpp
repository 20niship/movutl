#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image;

class CompoRefEntt final : public Entity {
public:
  Vec3 pos;                        // MPROPERTY(name="位置", viewer_anchor=true, position=true)
  Vec2 scale     = Vec2(1.0, 1.0); // MPROPERTY(name="拡大率", scale=true)
  float rotation = 0.0;            // MPROPERTY(name="回転", angle=true, radians=true)
  float alpha    = 1.0;            // MPROPERTY(name="透明度")
  int start_frame        = 0;      // MPROPERTY(name="開始フレーム", desc="参照先コンポジションの再生開始フレーム位置")
  float speed             = 1.0f;  // MPROPERTY(name="再生速度", min=0.0, max=10.0, step=0.1)
  uint32_t target_comp_guid = 0;   // MPROPERTY(name="参照コンポジション", hidden_inspector=true)

  virtual bool render(Composition* cmp, Image* target, int frame) override;
  virtual EntityType getType() const override { return EntityType_Scene; }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
