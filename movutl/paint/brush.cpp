#include <algorithm>
#include <cmath>
#include <movutl/paint/brush.hpp>
#include <stb_image.h>

namespace mu {

void CircleTip::render(const TipParams& p, int w, int h, float* mask) const {
  const float inner = p.radius * std::clamp(hardness_, 0.0f, 1.0f);
  const float edge  = std::max(p.radius - inner, 1.0f); // 最低1pxはぼかしてアンチエイリアスにする
  for(int j = 0; j < h; j++) {
    for(int i = 0; i < w; i++) {
      const float d = std::hypot(i + 0.5f - p.cx, j + 0.5f - p.cy);
      if(d < p.radius) mask[j * w + i] = std::clamp((p.radius - d) / edge, 0.0f, 1.0f);
    }
  }
}

void ImageTip::render(const TipParams& p, int w, int h, float* mask) const {
  if(!img_ || img_->empty() || p.radius <= 0) return;
  const float ca = std::cos(p.angle), sa = std::sin(p.angle);
  const int iw = img_->width(), ih = img_->height();
  for(int j = 0; j < h; j++) {
    for(int i = 0; i < w; i++) {
      // 画面座標 → 先端ローカル座標(-1〜1)。angleだけ逆回転する
      const float dx = (i + 0.5f - p.cx) / p.radius, dy = (j + 0.5f - p.cy) / p.radius;
      const float u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
      if(u < -1 || u >= 1 || v < -1 || v >= 1) continue;
      const Vec4b& c  = (*img_)((int)((u + 1) * 0.5f * iw), (int)((v + 1) * 0.5f * ih));
      const float lum = (0.299f * c[0] + 0.587f * c[1] + 0.114f * c[2]) / 255.0f;
      mask[j * w + i] = (1.0f - lum) * c[3] / 255.0f;
    }
  }
}

std::shared_ptr<PixelBuffer8> load_brush_image(const std::string& path) {
  int w = 0, h = 0, n = 0;
  unsigned char* d = stbi_load(path.c_str(), &w, &h, &n, 4);
  if(!d) return nullptr;
  auto img = std::make_shared<PixelBuffer8>(w, h);
  for(size_t i = 0; i < img->size(); i++) img->data()[i] = Vec4b{d[i * 4], d[i * 4 + 1], d[i * 4 + 2], d[i * 4 + 3]};
  stbi_image_free(d);
  return img;
}

float BrushTexture::sample(int x, int y) const {
  if(!img || img->empty() || depth <= 0) return 1.0f;
  const float s = std::max(scale, 0.01f);
  auto wrap     = [](float v, int n) {
    int i = (int)std::floor(v) % n;
    return i < 0 ? i + n : i;
  };
  const Vec4b& c  = (*img)(wrap(x / s, img->width()), wrap(y / s, img->height()));
  const float lum = (0.299f * c[0] + 0.587f * c[1] + 0.114f * c[2]) / 255.0f;
  return 1.0f - depth * (1.0f - lum);
}

} // namespace mu
