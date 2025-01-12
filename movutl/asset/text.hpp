#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image;

class TextEntt final : public Entity {
public:
  TextEntt() = default;
  TextEntt(const char* path);
  ~TextEntt() = default;

  Ref<Image> img_;

  Vec3 pos_;             // MPROPERTY(name="位置" viewer_anchor=true, position=true)
  float scale_x_ = 1.0f; // MPROPERTY(name="拡大率X, scale_x")
  float scale_y_ = 1.0f; // MPROPERTY(name="拡大率Y, scale_y")
  float rot_;            // MPROPERTY(name="回転", angle=true, radians=true)
  float speed = 100.0f;  // MPROPERTY(name="再生速度")
  uint8_t alpha_ = 255;  // MPROPERTY(name="透明度")
  std::string font;      // MPROPERTY(name="フォント", type="font")
  std::string text;      // MPROPERTY(name="テキスト")
  bool separate = false; // MPROPERTY(name="個別オブジェクト")

  static Ref<TextEntt> Create(const char* text, const char* font = nullptr);
  virtual EntityType getType() const override { return EntityType_3DText; }
  virtual bool render(Composition* cmp) override;

  virtual PropsInfo getPropsInfo() const override;    // MUFUNC_AUTOGEN
  virtual Props getProps() const override;            // MUFUNC_AUTOGEN
  virtual void setProps(const Props& props) override; // MUFUNC_AUTOGEN
};

} // namespace mu
