#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image;

class TextEntt final : public Entity {
private:
  std::string last_text_;
  std::string last_font_;
  Vec4b last_color_;
  Vec4b last_border_color_;
  int32_t last_border_width_ = -1;
  Vec2 text_offset_; // 枠線用に余白を足した分、表示位置を補正するオフセット

  void re_render_image();

public:
  TextEntt() = default;
  TextEntt(const char* path);
  ~TextEntt() = default;

  Ref<Image> img_;
  int32_t dirty_ = 0;                                // MPROPERTY(name="更新フラグ", hidden=true)
  Vec3 pos_;                                         // MPROPERTY(name="位置" viewer_anchor=true, position=true)
  float scale_x_ = 1.0;                              // MPROPERTY(name="拡大率X, scale_x")
  float scale_y_ = 1.0;                              // MPROPERTY(name="拡大率Y, scale_y")
  float rot_;                                        // MPROPERTY(name="回転", angle=true, radians=true)
  float speed    = 100.0;                            // MPROPERTY(name="再生速度")
  uint8_t alpha_ = 255;                              // MPROPERTY(name="透明度")
  std::string font;                                  // MPROPERTY(name="フォント", type="font")
  std::string text;                                  // MPROPERTY(name="テキスト")
  bool separate         = false;                     // MPROPERTY(name="個別オブジェクト")
  Vec4b color_          = Vec4b(255, 255, 255, 255); // MPROPERTY(name="文字色")
  Vec4b border_color_   = Vec4b(0, 0, 0, 255);       // MPROPERTY(name="枠線の色")
  int32_t border_width_ = 0;                         // MPROPERTY(name="枠線の太さ(0で非表示)")

  static Ref<TextEntt> Create(const char* text, const char* font = nullptr);
  virtual EntityType getType() const override { return EntityType_3DText; }
  virtual bool render(Composition* cmp) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
