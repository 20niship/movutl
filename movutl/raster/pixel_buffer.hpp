#pragma once

#include <cstddef>
#include <cstdint>
#include <movutl/core/assert.hpp>
#include <movutl/core/vector.hpp>
#include <vector>

namespace mu {

using Vec4w = _Vec<uint16_t, 4>; // 16bit/チャンネルRGBA

// ペイント層向けのピクセル格納(Entityとは独立)。Pxは1ピクセルの型で、チャンネル深度ごとにインスタンス化する
// ponytail: 当面は8bit(PixelBuffer8)のみ実用。PixelBuffer16は将来用のプレースホルダで、blend等のカーネルは未対応
template <typename Px> class PixelBuffer {
  std::vector<Px> data_;
  int w_ = 0, h_ = 0;

public:
  using pixel_type = Px;

  PixelBuffer() = default;
  PixelBuffer(int w, int h) { resize(w, h); }

  void resize(int w, int h) {
    MU_ASSERT(w >= 0 && h >= 0);
    w_ = w;
    h_ = h;
    data_.assign((size_t)w * h, Px());
  }
  void fill(const Px& c) { data_.assign(data_.size(), c); }

  int width() const { return w_; }
  int height() const { return h_; }
  size_t size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }
  static constexpr size_t bytes_per_pixel() { return sizeof(Px); }
  size_t size_in_bytes() const { return data_.size() * sizeof(Px); }

  Px* data() { return data_.data(); }
  const Px* data() const { return data_.data(); }

  Px& operator()(int x, int y) {
    MU_ASSERT(x >= 0 && x < w_ && y >= 0 && y < h_);
    return data_[(size_t)y * w_ + x];
  }
  const Px& operator()(int x, int y) const {
    MU_ASSERT(x >= 0 && x < w_ && y >= 0 && y < h_);
    return data_[(size_t)y * w_ + x];
  }
};

using PixelBuffer8  = PixelBuffer<Vec4b>; // 8bit RGBA(標準)
using PixelBuffer16 = PixelBuffer<Vec4w>; // 16bit RGBA(プレースホルダ)

} // namespace mu
