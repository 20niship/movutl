#pragma once

#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image : public Entity {
public:
  enum Format {
    FormatRGB = 0,
    FormatRGBA = 2,
    FormatGRAYSCALE = 1,
    FormatBGR = 3,
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
  void reset() {
    width_ = 0;
    height_ = 0;
    data_.clear();
  }
  void fill(const uint8_t& v) {
    for(size_t i = 0; i < size(); i++) data_[i] = v;
  }

  void resize(const Vec2d& size) {
    MU_ASSERT(size[0] > 0 && size[1] > 0);
    width_ = size[0];
    height_ = size[1];
    this->reserve(this->size());
  }
  void resize(const int _w, const int _h) { resize({_w, _h}); }

  const uint8_t& operator[](size_t i) const {
    MU_ASSERT(i < size());
    return data_[i];
  }
  uint8_t& operator[](size_t i) {
    assert(i < size());
    return data_[i];
  }
  const uint8_t& operator()(const size_t x, const size_t y) const {
    MU_ASSERT(x * y < size());
    return data_[y * width_ + x];
  }
  uint8_t& operator()(const size_t x, const size_t y) {
    MU_ASSERT(x * y < size());
    return data_[y * width_ + x];
  }
};

} // namespace mu
