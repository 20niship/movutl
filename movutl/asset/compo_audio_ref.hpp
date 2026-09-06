#pragma once

#include <movutl/asset/entity.hpp>

namespace mu {

class CompoAudioEntt final : public Entity {
public:
  int start_frame           = 0;    // MPROPERTY(name="開始フレーム")
  float speed               = 1.0f; // MPROPERTY(name="再生速度", min=0.0, max=10.0, step=0.1)
  float volume_             = 100.0f; // MPROPERTY(name="音量", min=0.0, max=200.0)
  uint32_t target_comp_guid = 0;    // MPROPERTY(name="参照コンポジション", hidden_inspector=true)

  virtual EntityType getType() const override { return EntityType_SceneAudio; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;
  virtual bool fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
