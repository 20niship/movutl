#pragma once
#include <movutl/asset/entity.hpp>

namespace mu {

class Image;

// AviUtlの「フレームバッファオブジェクト」相当。自分より下のレイヤーの合成結果をキャプチャし、指定位置/拡大率/回転で貼り戻す
class FramebufferEntt final : public Entity {
private:
  Ref<Image> captured_; // キャプチャした画像

public:
  FramebufferEntt()  = default;
  ~FramebufferEntt() = default;

  bool clear_original_ = false;          // MPROPERTY(name="元のバッファをクリア", desc="キャプチャ後に合成先バッファを透明でクリアする")
  Vec3 pos_            = Vec3(0, 0, 0);  // MPROPERTY(name="位置", viewer_anchor=true)
  Vec2 scale_          = Vec2(100, 100); // MPROPERTY(name="拡大率", scale=true)
  float rotation_      = 0;              // MPROPERTY(name="回転")
  uint8_t alpha_       = 255;            // MPROPERTY(name="透明度")

  static Ref<FramebufferEntt> Create(const char* name);
  virtual EntityType getType() const override { return EntityType_Framebuffer; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;

  // キャプチャした画像(nullptrの場合は未キャプチャ)
  const Ref<Image>& captured_image() const { return captured_; }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
