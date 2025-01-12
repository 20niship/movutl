#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace cv {
class Mat;
}

namespace mu {

class Image final : public Entity {
public:
private:
  ImageFormat fmt = ImageFormatRGB;
  Vec<uint8_t> data_;

  void reserve(size_t new_capacity) { data_.resize(new_capacity); }

public:
  Image() = default;
  ~Image() = default;

  unsigned int width = 0;  // MPROPERTY(name="幅", readonly=true)
  unsigned int height = 0; // MPROPERTY(name="高さ", readonly=true)
  Vec3 pos;                // MPROPERTY(name="位置" viewer_anchor=true, position=true)
  float scale_x = 1.0f;    // MPROPERTY(name="拡大率X, scale_x)
  float scale_y = 1.0f;    // MPROPERTY(name="拡大率Y, scale_y)
  float rotation = 0.0f;   // MPROPERTY(name="回転", angle=true, radians=true)
  float alpha = 1.0f;      // MPROPERTY(name="透明度")
  std::string path;        // MPROPERTY(name="ファイル", type="path")

  void set_rgb(const size_t x, const size_t y, const Vec3b& rgb) {
    MU_ASSERT(channels() >= 3);
    auto* ptr = &data_[y * width * 3 + x * 3];
    ptr[0] = rgb[0];
    ptr[1] = rgb[1];
    ptr[2] = rgb[2];
    if(channels() == 4) ptr[3] = 255;
  }
  void set_rgba(const size_t x, const size_t y, const Vec4b& rgba) {
    int c = channels();
    auto* ptr = &data_[(y * width + x) * c];
    if(c == 3) {
      ptr[0] = rgba[0];
      ptr[1] = rgba[1];
      ptr[2] = rgba[2];
    } else if(c == 4) {
      ptr[0] = rgba[0];
      ptr[1] = rgba[1];
      ptr[2] = rgba[2];
      ptr[3] = rgba[3];
    }
    ptr[0] = rgba[0];
  }

  bool copyto(Image* dst, const Vec2d& pmin) const;
  bool copyto(Image* dst, const Vec2d& pmin, float scale, const Vec2d& offset) const;
  bool copyto(Image* dst, const Vec2d& pmin, const Vec2d& pmax) const;
  bool copyto(Image* dst, const Vec2d& center, float scale, float angle) const;

  uint8_t* data() { return data_.data(); }
  size_t size() const { return width * height; }
  size_t size_in_bytes() const { return size() * channels(); }
  void reset() {
    width = 0;
    height = 0;
    data_.clear();
  }
  void fill(const uint8_t& v) { std::memset(data_.data(), v, size_in_bytes()); }

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
    width = size[0];
    height = size[1];
    data_.resize(size_in_bytes());
  }
  void resize(const int _w, const int _h) { resize({_w, _h}); }

  template <typename T> T at(int x, int y) const {
    MU_ASSERT(x >= 0 && x < (int)width && y >= 0 && y < (int)height);
    auto ptr = &data_[(y * width + x) * sizeof(T)];
    return *(T*)ptr;
  }

  const uint8_t& operator[](size_t i) const {
    MU_ASSERT(i < size());
    return data_[i];
  }
  uint8_t& operator[](size_t i) {
    MU_ASSERT(i < size());
    return data_[i];
  }

  Vec4b rgba(const size_t x, const size_t y) const {
    MU_ASSERT(x < width);
    MU_ASSERT(y < height);
    int c = channels();
    auto* ptr = &data_[(y * width + x) * c];
    if(c == 3) return {ptr[0], ptr[1], ptr[2], 255};
    if(c == 4) return {ptr[0], ptr[1], ptr[2], ptr[3]};
    return {ptr[0], ptr[0], ptr[0], 255};
  }

  const uint8_t& operator()(const size_t x, const size_t y) const {
    MU_ASSERT(x * y < size());
    return data_[(y * width + x) * channels()];
  }
  uint8_t& operator()(const size_t x, const size_t y) {
    MU_ASSERT(x * y < size());
    return data_[(y * width + x) * channels()];
  }

  void set_cv_img(const cv::Mat* cv_img);
  void to_cv_img(cv::Mat* cv_img) const;
  void imshow(const char* name = "img") const;
  virtual bool render(Composition* cmp) override;
  virtual EntityType getType() const override { return EntityType_Image; }

  static Ref<Image> Create(const char* name, const char* path, bool add_to_pj = true);
  static Ref<Image> Create(const char* name, int w, int h, ImageFormat format = ImageFormatRGB, bool add_to_pj = true);
};

void cv_waitkey(int time = 0);
} // namespace mu
