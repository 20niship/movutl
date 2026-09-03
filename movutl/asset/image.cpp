#include <algorithm>
#include <cstring>
#include <opencv2/opencv.hpp>
//
#include <movutl/app/app.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/core/time.hpp>
#include <movutl/plugin/input.hpp>

namespace mu {

void Image::fill(const uint32_t& v) { std::memset(data_.data(), v, size_in_bytes()); }

void Image::fill_rgba(const Vec4b& c) {
  MOVUTL_ZONE_SCOPED_N("Image::fill_rgba");
  static_assert(sizeof(Vec4b) == sizeof(uint32_t), "Vec4b size must match uint32_t");
  uint32_t packed;
  std::memcpy(&packed, &c, sizeof(uint32_t));
  std::fill_n(reinterpret_cast<uint32_t*>(data()), size(), packed);
}

void Image::set_cv_img(const cv::Mat* cv_img) {
  MU_ASSERT(cv_img);
  cv::Mat out;
  if(cv_img->channels() == 4)
    out = *cv_img;
  else
    cv::cvtColor(*cv_img, out, cv::COLOR_BGR2BGRA);

  MU_ASSERT(cv_img->rows > 0 && cv_img->cols > 0);
  MU_ASSERT(cv_img->channels() == 1 || cv_img->channels() == 3 || cv_img->channels() == 4);
  MU_ASSERT(out.channels() == 4);

  resize(cv_img->cols, cv_img->rows);
  for(int y = 0; y < cv_img->rows; y++) {
    const uint32_t* cv_ptr = reinterpret_cast<const uint32_t*>(cv_img->ptr<uint8_t>(y));
    uint32_t* dst_p        = reinterpret_cast<uint32_t*>(&data_[y * width]);
    /*for(int x = 0; x < cv_img->cols; x++) dst_p[x] = cv_ptr[x];*/
    std::memcpy(dst_p, cv_ptr, cv_img->cols * 4);
  }
}

void Image::to_cv_img(cv::Mat* cv_img) const {
  MU_ASSERT(cv_img);
  cv_img->create(height, width, CV_8UC4);
  for(int y = 0; y < (int)height; y++) {
    uint32_t* cv_ptr      = cv_img->ptr<uint32_t>(y);
    const uint32_t* src_p = reinterpret_cast<const uint32_t*>(&data_[0] + y * width);
    std::memcpy(cv_ptr, src_p, width * 4);
    /*for(int x = 0; x < (int)width; x++) cv_ptr[x] = src_p[x];*/
  }
}

namespace {
// BlendTypeに応じたチャンネル合成式(0-255域)。dはブレンド前の合成先(base)、sはブレンド元(blend)の値
inline uint8_t blend_channel(BlendType mode, uint8_t d, uint8_t s) {
  switch(mode) {
    case Blend_Add: return (uint8_t)std::min(255, (int)d + (int)s);
    case Blend_Sub: return (uint8_t)std::max(0, (int)d - (int)s);
    case Blend_Mul: return (uint8_t)((int)d * (int)s / 255);
    case Blend_Div: return s > 0 ? (uint8_t)std::min(255, (int)d * 255 / (int)s) : 255;
    case Blend_Screen: return (uint8_t)(255 - (255 - d) * (255 - s) / 255);
    case Blend_Overlay: return d < 128 ? (uint8_t)(2 * d * s / 255) : (uint8_t)(255 - 2 * (255 - d) * (255 - s) / 255);
    case Blend_Darken: return std::min(d, s);
    case Blend_Lighten: return std::max(d, s);
    case Blend_HardLight: return s < 128 ? (uint8_t)(2 * d * s / 255) : (uint8_t)(255 - 2 * (255 - d) * (255 - s) / 255);
    case Blend_Alpha:
    default: return s;
  }
}

/// srcをdstへ合成する。alpha=255(かつalpha_mul=1、blend=Blend_Alpha)なら単純上書きと同じ結果になる
inline void blend_pixel(Vec4b& d, const Vec4b& src, float alpha_mul, BlendType blend = Blend_Alpha) {
  float a = (src[3] / 255.0f) * alpha_mul;
  if(a <= 0.0f) return;
  if(blend == Blend_Alpha) {
    if(a >= 1.0f) {
      d = src;
      return;
    }
    for(int c = 0; c < 3; c++) d[c] = (unsigned char)(src[c] * a + d[c] * (1.0f - a));
    d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
    return;
  }
  for(int c = 0; c < 3; c++) {
    uint8_t blended = blend_channel(blend, d[c], src[c]);
    d[c]            = (unsigned char)(blended * a + d[c] * (1.0f - a));
  }
  d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
}
} // namespace

bool Image::copyto(Image* dst, const Vec2d& pmin, float alpha_mul, BlendType blend) const {
  MOVUTL_ZONE_SCOPED_N("Image::copyto(pmin)");
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;
  int cw = dst->width;
  int ch = dst->height;

  int px = (int)pmin[0];
  int py = (int)pmin[1];
  // 事前にクリップ範囲を求め、内側ループの毎ピクセル境界チェックを無くす
  int x0 = std::max(0, -px);
  int x1 = std::min((int)this->width, cw - px);
  int y0 = std::max(0, -py);
  int y1 = std::min((int)this->height, ch - py);
  if(x0 >= x1 || y0 >= y1) return true;

  // alphaを考慮しない(不透明かつalpha_mul=1、通常合成)なら行単位memcpyで済む
  bool opaque_copy = !this->has_alpha && alpha_mul >= 1.0f && blend == Blend_Alpha;
  int row_w        = x1 - x0;
  for(int y = y0; y < y1; y++) {
    Vec4b* dst_row       = &dst->data_[(py + y) * cw + (px + x0)];
    const Vec4b* src_row = &data_[y * width + x0];
    if(opaque_copy) {
      std::memcpy(dst_row, src_row, row_w * sizeof(Vec4b));
    } else {
      for(int x = 0; x < row_w; x++) blend_pixel(dst_row[x], src_row[x], alpha_mul, blend);
    }
  }
  return true;
}

bool Image::copyto(Image* dst, const Vec2d& center, float scale, float angle, float alpha_mul, BlendType blend) const {
  MOVUTL_ZONE_SCOPED_N("Image::copyto(center,scale,angle)");
  if(angle == 0 && scale == 1.0) return this->copyto(dst, center, alpha_mul, blend);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;
  // centerはpmin相当。回転/拡大の軸はdst全体の中心ではなく画像自身の中心にする(旧実装は中心からズレた位置で意図せず移動して見えるバグがあった)
  float rad     = angle * M_PI / 180.0f;
  float cos_a   = std::cos(rad);
  float sin_a   = std::sin(rad);
  float true_cx = center[0] + this->width / 2.0f;
  float true_cy = center[1] + this->height / 2.0f;
  float src_cx  = this->width / 2.0f;
  float src_cy  = this->height / 2.0f;

  // srcの4隅をdst空間へ順変換し、影響範囲のバウンディングボックスだけ走査する(旧実装は毎回dst全域を走査していた)
  float half_w      = this->width / 2.0f * scale;
  float half_h      = this->height / 2.0f * scale;
  float corner_x[4] = {-half_w, half_w, -half_w, half_w};
  float corner_y[4] = {-half_h, -half_h, half_h, half_h};
  float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
  for(int i = 0; i < 4; i++) {
    float bx = true_cx + corner_x[i] * cos_a - corner_y[i] * sin_a;
    float by = true_cy + corner_x[i] * sin_a + corner_y[i] * cos_a;
    min_x    = std::min(min_x, bx);
    max_x    = std::max(max_x, bx);
    min_y    = std::min(min_y, by);
    max_y    = std::max(max_y, by);
  }
  int bbox_x0 = std::max(0, (int)std::floor(min_x));
  int bbox_x1 = std::min((int)dst->width, (int)std::ceil(max_x));
  int bbox_y0 = std::max(0, (int)std::floor(min_y));
  int bbox_y1 = std::min((int)dst->height, (int)std::ceil(max_y));

  for(int y = bbox_y0; y < bbox_y1; ++y) {
    for(int x = bbox_x0; x < bbox_x1; ++x) {
      // dst上のこのピクセルが、画像自身の中心を軸とした逆回転・逆拡大でsrcのどこに対応するか
      float dx    = x - true_cx;
      float dy    = y - true_cy;
      float src_x = src_cx + (dx * cos_a + dy * sin_a) / scale;
      float src_y = src_cy + (-dx * sin_a + dy * cos_a) / scale;

      // 元画像の座標が範囲内か確認
      int src_x_int = static_cast<int>(std::floor(src_x));
      if(src_x_int < 0 || src_x_int >= this->width) continue;
      int src_y_int = static_cast<int>(std::floor(src_y));
      if(src_y_int < 0 || src_y_int >= this->height) continue;

      blend_pixel(dst->data_[y * dst->width + x], data_[src_y_int * width + src_x_int], alpha_mul, blend);
    }
  }
  return true;
}

bool Image::drawpoly(Image* dst, const Vec2d corners[4], float alpha_mul, BlendType blend) const {
  MOVUTL_ZONE_SCOPED_N("Image::drawpoly");
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;

  // corners順は左上,右上,左下,右下(AviUtl obj.drawpolyの引数順に合わせる)
  cv::Point2f src_pts[4] = {{0, 0}, {(float)width, 0}, {0, (float)height}, {(float)width, (float)height}};
  cv::Point2f dst_pts[4];
  float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
  for(int i = 0; i < 4; i++) {
    dst_pts[i] = cv::Point2f((float)corners[i][0], (float)corners[i][1]);
    min_x      = std::min(min_x, (float)corners[i][0]);
    max_x      = std::max(max_x, (float)corners[i][0]);
    min_y      = std::min(min_y, (float)corners[i][1]);
    max_y      = std::max(max_y, (float)corners[i][1]);
  }
  cv::Mat fwd = cv::getPerspectiveTransform(src_pts, dst_pts);
  cv::Mat inv;
  cv::invert(fwd, inv);

  int bbox_x0 = std::max(0, (int)std::floor(min_x));
  int bbox_x1 = std::min((int)dst->width, (int)std::ceil(max_x));
  int bbox_y0 = std::max(0, (int)std::floor(min_y));
  int bbox_y1 = std::min((int)dst->height, (int)std::ceil(max_y));

  // dst上の各画素から逆射影変換でsrc座標を求め、範囲内ならblend_pixelで合成する(既存BlendTypeに対応するためcv::warpPerspectiveは使わない)
  const double* m = inv.ptr<double>();
  for(int y = bbox_y0; y < bbox_y1; ++y) {
    for(int x = bbox_x0; x < bbox_x1; ++x) {
      double w = m[6] * x + m[7] * y + m[8];
      if(std::abs(w) < 1e-9) continue;
      double src_x = (m[0] * x + m[1] * y + m[2]) / w;
      double src_y = (m[3] * x + m[4] * y + m[5]) / w;
      int sx       = (int)std::floor(src_x);
      int sy       = (int)std::floor(src_y);
      if(sx < 0 || sx >= (int)this->width || sy < 0 || sy >= (int)this->height) continue;
      blend_pixel(dst->data_[y * dst->width + x], data_[sy * width + sx], alpha_mul, blend);
    }
  }
  return true;
}

void Image::outline(const Vec4b& border_color, int border_width) {
  MOVUTL_ZONE_SCOPED_N("Image::outline");
  if(border_width <= 0 || this->width == 0 || this->height == 0) return;

  cv::Mat alpha_mat((int)this->height, (int)this->width, CV_8UC1);
  for(size_t y = 0; y < this->height; y++)
    for(size_t x = 0; x < this->width; x++) alpha_mat.at<uint8_t>((int)y, (int)x) = (*this)(x, y)[3];

  cv::Mat dilated;
  int k = border_width * 2 + 1;
  cv::dilate(alpha_mat, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));

  for(size_t y = 0; y < this->height; y++) {
    for(size_t x = 0; x < this->width; x++) {
      Vec4b& px = (*this)(x, y);
      if(px[3] > 0) continue; // 塗りがある部分はそのまま残す
      uint8_t d = dilated.at<uint8_t>((int)y, (int)x);
      if(d == 0) continue;
      px = Vec4b(border_color[0], border_color[1], border_color[2], d);
    }
  }
}

bool Image::render(Composition* cmp, Image* target, int frame) {
  (void)frame;
  MU_ASSERT(cmp);
  MU_ASSERT(target);
  if(this->width <= 0 || this->height <= 0) return false;
  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  int base_x = (int)this->pos[0];
  int base_y = (int)this->pos[1];

  const Image* src = this;
  if(!trk.filters.empty()) {
    if(!filtered_) filtered_ = cutil::make_ref<Image>();
    filtered_->resize(this->width, this->height);
    filtered_->has_alpha = this->has_alpha;
    filtered_->fmt       = this->fmt;
    std::memcpy(filtered_->data(), this->data_.data(), this->size_in_bytes());
    render_filters(cmp, filtered_.get(), frame);
    src = filtered_.get();
  }
  return src->copyto(target, Vec2d(base_x, base_y), this->scale.avg(), this->rotation, this->alpha, trk.blend_);
}

bool Image::load_file(const char* path) {
  if(in_plg_ != nullptr && in_handle_ != nullptr) in_plg_->fn_close(in_handle_);
  in_plg_    = nullptr;
  in_handle_ = nullptr;
  info       = EntityInfo();

  auto p = get_compatible_plugin(path, EntityType_Image);
  if(p == nullptr) {
    LOG_F(ERROR, "No compatible image plugin found for file: %s", path);
    return false;
  }
  this->path = path;

  InputHandle h = p->fn_open(path);
  if(h == nullptr) {
    LOG_F(ERROR, "Failed to open image file: %s", path);
    return false;
  }
  if(p->fn_info_get == nullptr || !p->fn_info_get(h, &info) || info.width <= 0 || info.height <= 0) {
    LOG_F(ERROR, "Failed to get image info: %s", path);
    p->fn_close(h);
    return false;
  }

  this->fmt = info.format;
  this->resize((int)info.width, (int)info.height);

  MU_ASSERT(p->fn_read_video);
  if(p->fn_read_video(h, 0, this->data()) <= 0) {
    LOG_F(ERROR, "Failed to read image data: %s", path);
    p->fn_close(h);
    reset();
    return false;
  }
  p->fn_close(h); // 静止画は一括読み込みで完結するのでハンドルを保持し続けない(Movieとの違い)
  LOG_F(INFO, "Image loaded: %s (%dx%d, plugin=%s)", path, info.width, info.height, p->name);
  return true;
}

Ref<Image> Image::Create(const char* name, const char* path) {
  MU_ASSERT(name);
  auto img  = cutil::make_ref<Image>();
  img->name = name;
  auto pj   = Project::Get();
  MU_ASSERT(pj);
  pj->entities.push_back(img);
  if(path && path[0] != '\0') img->load_file(path);
  return img;
}

Ref<Image> Image::Create(const char* name, int w, int h, ImageFormat format, bool add_to_pj) {
  MU_ASSERT(name && w > 0 && h > 0);
  auto img = cutil::make_ref<Image>();
  MU_ASSERT(img);
  img->name = name;
  img->fmt  = format;
  img->resize(w, h);
  if(add_to_pj) {
    auto pj = Project::Get();
    MU_ASSERT(pj);
    pj->entities.push_back(img);
  }
  return img;
}

void Image::imshow(const char* name) const {
  cv::Mat cv_img;
  to_cv_img(&cv_img);
  cv::imshow(name, cv_img);
}

void cv_waitkey(int time) { cv::waitKey(time); }

} // namespace mu
