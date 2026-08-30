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

  Vec3 pos_; // 図形の左上を置く位置(comp座標系)
  Vec2 size_          = Vec2(200, 200);
  float rot_          = 0.0f; // radians
  uint8_t alpha_      = 255;
  Vec4b color_        = Vec4b(255, 255, 255, 255);
  int32_t shape_type_ = ShapeType_Rect;
  std::string custom_path; // "x,y;x,y;..." (shape_type_がShapeType_Customの時のみ使用)
  Vec4b border_color_   = Vec4b(0, 0, 0, 255);
  int32_t border_width_ = 0; // 0の場合枠線なし

  static Ref<ShapeEntt> Create(const char* name, ShapeType type = ShapeType_Rect);
  virtual EntityType getType() const override { return EntityType_Polygon; }
  virtual bool render(Composition* cmp) override;

  virtual const cutil::PropInfo* getPropsInfo() const override;
  virtual cutil::Prop getProps() const override;
  virtual void setProps(const cutil::Prop& props) override;
};

} // namespace mu
