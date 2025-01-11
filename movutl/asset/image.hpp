#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace cv {
class Mat;
}

namespace mu {

class Image : public Entity {
public:
  enum Format {
    FormatRGB = 0,
    FormatRGBA = 2,
    FormatGRAYSCALE = 1,
  };

private:
  Format _format = Format::FormatRGB;
  Vec<uint8_t> data_;

  void reserve(size_t new_capacity) { data_.resize(new_capacity); }

public:
  Image();
  Image(const char* image_file_name);
  Image(int w, int h, const Format format);
  Image(int w, int h, uint8_t* data, const Format format);
  ~Image();

  std::string path_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;

  uint8_t* data() { return data_.data(); }
  size_t size() const { return width_ * height_; }
  size_t size_in_bytes() const { return size() * channels(); }
  void reset() {
    width_ = 0;
    height_ = 0;
    data_.clear();
  }
  void fill(const uint8_t& v) {
    for(size_t i = 0; i < size(); i++) data_[i] = v;
  }

  int channels() const {
    switch(_format) {
      case Format::FormatRGB: return 3;
      case Format::FormatRGBA: return 4;
      case Format::FormatGRAYSCALE: return 1;
      default: MU_ASSERT(false);
    }
  }

  void resize(const Vec2d& size) {
    MU_ASSERT(size[0] > 0 && size[1] > 0);
    width_ = size[0];
    height_ = size[1];
    data_.resize(size_in_bytes());
  }
  void resize(const int _w, const int _h) { resize({_w, _h}); }

  template <typename T> T at(int x, int y) const {
    MU_ASSERT(x >= 0 && x < width_ && y >= 0 && y < height_);
    auto ptr = &data_[(y * width_ + x) * sizeof(T)];
    return *(T*)ptr;
  }

  const uint8_t& operator[](size_t i) const {
    MU_ASSERT(i < size());
    return data_[i];
  }
  uint8_t& operator[](size_t i) {
    assert(i < size());
    return data_[i];
  }
  Vec3b rgb(const size_t x, const size_t y) const {
    MU_ASSERT(channels() == 3);
    auto* ptr = &data_[y * width_ * 3 + x * 3];
    return {ptr[0], ptr[1], ptr[2]};
  }

  const uint8_t& operator()(const size_t x, const size_t y) const {
    MU_ASSERT(x * y < size());
    return data_[(y * width_ + x) * channels()];
  }
  uint8_t& operator()(const size_t x, const size_t y) {
    MU_ASSERT(x * y < size());
    return data_[(y * width_ + x) * channels()];
  }

  void set_cv_img(const cv::Mat* cv_img);
  void to_cv_img(cv::Mat* cv_img) const;
  void imshow() const;
};

void cv_waitkey(int time = 0);
} // namespace mu
