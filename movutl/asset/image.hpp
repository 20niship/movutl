#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/vector.hpp>

namespace cv {
class Mat;
}

namespace mu {

class Image final : public Entity {
private:
  Vec<Vec4b> data_;

  void reserve(size_t new_capacity) { data_.resize(new_capacity); }

public:
  Image() = default;
  Image(int w, int h) { resize(w, h); }
  ~Image() = default;

  unsigned int width  = 0; // MPROPERTY(name="幅", readonly=true)
  unsigned int height = 0; // MPROPERTY(name="高さ", readonly=true)
  int16_t dirty_      = 1;
  bool has_alpha      = true;

  ImageFormat fmt = ImageFormatRGBA; // MPROPERTY(name="フォーマット", readonly=true)
  Vec3 pos;                          // MPROPERTY(name="位置" viewer_anchor=true, position=true)
  Vec2 scale     = Vec2(1.0, 1.0);   // MPROPERTY(name="拡大率X, scale=true)
  float rotation = 0.0;              // MPROPERTY(name="回転", angle=true, radians=true)
  float alpha    = 1.0;              // MPROPERTY(name="透明度")
  std::string path;                  // MPROPERTY(name="ファイル", type="path")

  void dirty() { dirty_++; }
  int get_dirty() const { return dirty_; }
  Vec4b* data() { return data_.data(); }

  void set_rgb(const size_t x, const size_t y, const Vec3b& rgb) {
    auto* ptr = &data_[y * width + x];
    auto p    = reinterpret_cast<Vec3b*>(ptr);
    *p        = rgb;
  }

  void set_rgba(const size_t x, const size_t y, const Vec4b& rgba) {
    auto* ptr = &data_[y * width + x];
    auto p    = reinterpret_cast<Vec4b*>(ptr);
    static_assert(sizeof(Vec4b) == 4, "Vec4b size must be 4");
    *p = rgba;
  }

  size_t size() const { return width * height; }
  size_t size_in_bytes() const { return size() * 4; }
  void reset() {
    width  = 0;
    height = 0;
    data_.clear();
  }
  void fill(const uint32_t& v) { std::memset(data_.data(), v, size_in_bytes()); }
  void fill_rgba(const Vec4b& c) {
    MOVUTL_ZONE_SCOPED_N("Image::fill_rgba");
    for(size_t i = 0; i < size(); i++) (*this)[i] = c;
  }

  bool copyto(Image* dst, const Vec2d& pmin) const;
  bool copyto(Image* dst, const Vec2d& pmin, const Vec2d& pmax) const;
  bool copyto(Image* dst, const Vec2d& center, float scale, float angle) const;

  void resize(const Vec2d& size) {
    MU_ASSERT(size[0] > 0 && size[1] > 0);
    width  = size[0];
    height = size[1];
    data_.resize(size_in_bytes());
  }
  void resize(const int _w, const int _h) { resize({_w, _h}); }

  template <typename T> T at(int x, int y) const {
    MU_ASSERT(x >= 0 && x < (int)width && y >= 0 && y < (int)height);
    auto ptr = &data_[(y * width + x) * sizeof(T)];
    return *(T*)ptr;
  }

  const Vec4b& operator[](size_t i) const {
    MU_ASSERT(i < size());
    return data_[i];
  }
  Vec4b& operator[](size_t i) {
    MU_ASSERT(i < size());
    return data_[i];
  }

  Vec4b rgba(const size_t x, const size_t y) const {
    MU_ASSERT(x < width);
    MU_ASSERT(y < height);
    constexpr int c = 4; // channels;
    return data_[(y * width + x) * c];
  }

  const Vec4b& operator()(const size_t x, const size_t y) const {
    MU_ASSERT(x * y < size());
    return data_[y * width + x];
  }
  Vec4b& operator()(const size_t x, const size_t y) {
    MU_ASSERT(x * y < size());
    return data_[y * width + x];
  }

  void set_cv_img(const cv::Mat* cv_img);
  void to_cv_img(cv::Mat* cv_img) const;
  void imshow(const char* name = "img") const;
  bool empty() const { return width == 0 || height == 0; }

  int channels() const {
    switch(fmt) {
      case ImageFormatRGB: return 3;
      case ImageFormatRGBA: return 4;
      case ImageFormatGRAYSCALE: return 1;
      default: MU_ASSERT(false);
    }
  }

  virtual bool render(Composition* cmp) override;
  virtual EntityType getType() const override { return EntityType_Image; }

  static Ref<Image> Create(const char* name, const char* path = "");
  static Ref<Image> Create(const char* name, int w, int h, ImageFormat format = ImageFormatRGBA, bool add_to_pj = true);
  bool load_file(const char* path);
  virtual void reload_asset() override {
    if(!path.empty()) load_file(path.c_str());
  }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

void cv_waitkey(int time = 0);

} // namespace mu
