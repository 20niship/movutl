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
  Vec2 origin_offset_; // 画像中心から見たpos_の原点のずれ。プリセット図形は0(画像中心が原点)、custom_pathはパス座標の(0,0)が原点

  void re_render_image();

public:
  ShapeEntt()  = default;
  ~ShapeEntt() = default;

  Vec2 size_          = Vec2(200, 200);            // MPROPERTY(name="サイズ")
  Vec4b color_        = Vec4b(255, 255, 255, 255); // MPROPERTY(name="色")
  int32_t shape_type_ = ShapeType_Rect;            // MPROPERTY(name="種類(0:三角 1:四角 2:六角 3:円 4:カスタム)")
  std::string custom_path;                         // MPROPERTY(name="カスタムパス(座標を x1 y1;x2 y2;... で列挙)")
  Vec4b border_color_   = Vec4b(0, 0, 0, 255);     // MPROPERTY(name="枠線の色")
  int32_t border_width_ = 0;                       // MPROPERTY(name="枠線の太さ(0で非表示)")

  static Ref<ShapeEntt> Create(const char* name, ShapeType type = ShapeType_Rect);
  virtual EntityType getType() const override { return EntityType_Polygon; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
