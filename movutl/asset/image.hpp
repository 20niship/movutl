#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/imagebase.hpp>
#include <movutl/core/vector.hpp>

namespace cv {
class Mat;
}

namespace mu {

class Image final : public Entity {
public:
  Image() = default;
  ~Image() = default;

  ImageRGBA img;

  ImageFormat fmt = ImageFormatRGBA; // MPROPERTY(name="フォーマット", readonly=true)
  Vec3 pos;                          // MPROPERTY(name="位置" viewer_anchor=true, position=true)
  Vec2 scale = Vec2(1.0, 1.0);       // MPROPERTY(name="拡大率X, scale=true)
  float rotation = 0.0;              // MPROPERTY(name="回転", angle=true, radians=true)
  float alpha = 1.0;                 // MPROPERTY(name="透明度")
  std::string path;                  // MPROPERTY(name="ファイル", type="path")

  bool copyto(Image* dst, const Vec2d& pmin) const { return img.copyto(&dst->img, pmin); }
  bool copyto(Image* dst, const Vec2d& pmin, const Vec2d& pmax) const { return img.copyto(&dst->img, pmin, pmax); }
  bool copyto(Image* dst, const Vec2d& center, float scale, float angle) const { return img.copyto(&dst->img, center, scale, angle); }

  size_t size() const { return img.size(); }
  void resize(const int _w, const int _h) { resize({_w, _h}); }
  int width() const { return img.width; }
  int height() const { return img.height; }
  void reset() { img.reset(); }
  void fill(const uint32_t& v) { img.fill(v); }
  Vec4b* data() { return img.data(); }

  void dirty() { img.dirty(); }
  int get_dirty() const { return img.dirty_; }

  int channels() const {
    switch(fmt) {
      case ImageFormatRGB: return 3;
      case ImageFormatRGBA: return 4;
      case ImageFormatGRAYSCALE: return 1;
      default: MU_ASSERT(false);
    }
  }

  void resize(const Vec2d& size) {
    MU_ASSERT(size[0] > 0 && size[1] > 0);
    img.resize(size);
  }

  void set_cv_img(const cv::Mat* cv_img) { img.set_cv_img(cv_img); }
  void to_cv_img(cv::Mat* cv_img) const { img.to_cv_img(cv_img); }
  void imshow(const char* name = "img") const { img.imshow(name); }
  virtual bool render(Composition* cmp) override;
  virtual EntityType getType() const override { return EntityType_Image; }

  static Ref<Image> Create(const char* name, const char* path = "");
  static Ref<Image> Create(const char* name, int w, int h, ImageFormat format = ImageFormatRGBA, bool add_to_pj = true);

  inline Vec4b& operator[](const size_t i) { return img[i]; }
  inline const Vec4b& operator[](const size_t i) const { return img[i]; }

  virtual PropsInfo getPropsInfo() const override;    // MUFUNC_AUTOGEN
  virtual Props getProps() const override;            // MUFUNC_AUTOGEN
  virtual void setProps(const Props& props) override; // MUFUNC_AUTOGEN
};

} // namespace mu
