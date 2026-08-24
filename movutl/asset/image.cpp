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
/// srcをdstへアルファブレンドで合成する。alpha=255(かつalpha_mul=1)なら単純上書きと同じ結果になる
inline void blend_pixel(Vec4b& d, const Vec4b& src, float alpha_mul) {
  float a = (src[3] / 255.0f) * alpha_mul;
  if(a <= 0.0f) return;
  if(a >= 1.0f) {
    d = src;
    return;
  }
  for(int c = 0; c < 3; c++) d[c] = (unsigned char)(src[c] * a + d[c] * (1.0f - a));
  d[3] = (unsigned char)(a * 255.0f + d[3] * (1.0f - a));
}
} // namespace

bool Image::copyto(Image* dst, const Vec2d& pmin, float alpha_mul) const {
  MOVUTL_ZONE_SCOPED_N("Image::copyto(pmin)");
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;
  int cw = dst->width;
  int ch = dst->height;
  if(cw <= 0 || ch <= 0) return false;

  for(int y = 0; y < this->height; y++) {
    int dy = pmin[1] + y;
    if(dy < 0 || dy >= ch) continue;
    for(int x = 0; x < this->width; x++) {
      int dx = pmin[0] + x;
      if(dx < 0 || dx >= cw) continue;
      blend_pixel(dst->data_[dy * cw + dx], data_[y * width + x], alpha_mul);
    }
  }
  return true;
}

bool Image::copyto(Image* dst, const Vec2d& center, float scale, float angle, float alpha_mul) const {
  MOVUTL_ZONE_SCOPED_N("Image::copyto(center,scale,angle)");
  if(angle == 0 && scale == 1.0) return this->copyto(dst, center, alpha_mul);
  // 角度をラジアンに変換
  float rad = angle * M_PI / 180.0f;
  float cx  = center[0];
  float cy  = center[1];

  // 回転とスケールの逆行列成分を計算
  float inv_cos = std::cos(rad) / scale;
  float inv_sin = std::sin(rad) / scale;

  float offset_x = cx - inv_cos * cx + inv_sin * cy;
  float offset_y = cy - inv_sin * cx - inv_cos * cy;

  int dst_x = dst->width / 2;
  int dst_y = dst->height / 2;

  for(int y = 0; y < dst->height; ++y) {
    for(int x = 0; x < dst->width; ++x) {
      // 出力画像の座標を元画像の座標に変換
      float src_x = inv_cos * (x - dst_x) - inv_sin * (y - dst_y) + offset_x;
      float src_y = inv_sin * (x - dst_x) + inv_cos * (y - dst_y) + offset_y;

      // 元画像の座標が範囲内か確認
      int src_x_int = static_cast<int>(std::floor(src_x));
      if(src_x_int < 0 || src_x_int >= this->width) continue;
      int src_y_int = static_cast<int>(std::floor(src_y));
      if(src_y_int < 0 || src_y_int >= this->height) continue;

      blend_pixel(dst->data_[y * dst->width + x], data_[src_y_int * width + src_x_int], alpha_mul);
    }
  }
  return true;
}

bool Image::render(Composition* cmp) {
  MU_ASSERT(cmp);
  MU_ASSERT(cmp->frame_final);
  if(this->width <= 0 || this->height <= 0) return false;
  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  int base_x = this->width / 2 + trk.anchor[0] - cw / 2;
  int base_y = this->height / 2 + trk.anchor[1] - ch / 2;
  return this->copyto(cmp->frame_final.get(), Vec2d(base_x, base_y));
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
