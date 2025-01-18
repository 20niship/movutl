#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace cv {
class Mat;
}

namespace mu {

class ImageRGBA {
private:
  Vec<Vec4b> data_;

  void reserve(size_t new_capacity) { data_.resize(new_capacity); }

public:
  ImageRGBA() = default;
  ~ImageRGBA() = default;

  unsigned int width = 0;  // MPROPERTY(name="幅", readonly=true)
  unsigned int height = 0; // MPROPERTY(name="高さ", readonly=true)
  int16_t dirty_ = 1;
  bool alpha = true;

  void dirty() { dirty_++; }
  Vec4b* data() { return data_.data(); }

  void set_rgb(const size_t x, const size_t y, const Vec3b& rgb) {
    auto* ptr = &data_[y * width + x];
    auto p = reinterpret_cast<Vec3b*>(ptr);
    *p = rgb;
  }

  void set_rgba(const size_t x, const size_t y, const Vec4b& rgba) {
    auto* ptr = &data_[y * width + x];
    auto p = reinterpret_cast<Vec4b*>(ptr);
    static_assert(sizeof(Vec4b) == 4, "Vec4b size must be 4");
    *p = rgba;
  }

  size_t size() const { return width * height; }
  size_t size_in_bytes() const { return size() * 4; }
  void reset() {
    width = 0;
    height = 0;
    data_.clear();
  }
  void fill(const uint32_t& v) { std::memset(data_.data(), v, size_in_bytes()); }

  bool copyto(ImageRGBA* dst, const Vec2d& pmin) const;
  bool copyto(ImageRGBA* dst, const Vec2d& pmin, const Vec2d& pmax) const;
  bool copyto(ImageRGBA* dst, const Vec2d& center, float scale, float angle) const;

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
};

void cv_waitkey(int time = 0);
} // namespace mu
