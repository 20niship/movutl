#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image;

// AviUtlの「図形」オブジェクト相当。三角形/四角形/六角形/円/任意パス(多角形)を描画する
class ShapeEntt final : public Entity {
private:
  Ref<Image> img_;
  int32_t last_type_ = -1;
  Vec2 last_size_;
  Vec4b last_color_;
  std::string last_path_;
  Vec4b last_border_color_;
  int32_t last_border_width_ = -1;
  Vec2 shape_offset_; // custom_path使用時、点群のbboxが(0,0)始まりでない場合のオフセット

  void re_render_image();

public:
  ShapeEntt()  = default;
  ~ShapeEntt() = default;

  Vec3 pos_;                                       // MPROPERTY(name="位置")
  Vec2 size_          = Vec2(200, 200);            // MPROPERTY(name="サイズ")
  float rot_          = 0.0f;                      // MPROPERTY(name="回転")
  uint8_t alpha_      = 255;                       // MPROPERTY(name="透明度")
  Vec4b color_        = Vec4b(255, 255, 255, 255); // MPROPERTY(name="色")
  int32_t shape_type_ = ShapeType_Rect;            // MPROPERTY(name="種類(0:三角 1:四角 2:六角 3:円 4:カスタム)")
  std::string custom_path;                         // MPROPERTY(name="カスタムパス(座標を x1 y1;x2 y2;... で列挙)")
  Vec4b border_color_   = Vec4b(0, 0, 0, 255);     // MPROPERTY(name="枠線の色")
  int32_t border_width_ = 0;                       // MPROPERTY(name="枠線の太さ(0で非表示)")

  static Ref<ShapeEntt> Create(const char* name, ShapeType type = ShapeType_Rect);
  virtual EntityType getType() const override { return EntityType_Polygon; }
  virtual bool render(Composition* cmp) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
