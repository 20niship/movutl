#include <opencv2/opencv.hpp>
//
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/time.hpp>

namespace mu {

void Image::set_cv_img(const cv::Mat* cv_img) {
  MU_ASSERT(cv_img);
  if(cv_img->channels() == 1)
    this->fmt = ImageFormatGRAYSCALE;
  else if(cv_img->channels() == 3)
    this->fmt = ImageFormatRGB;
  else if(cv_img->channels() == 4)
    this->fmt = ImageFormatRGBA;
  else
    MU_ASSERT(false);

  MU_ASSERT(cv_img->rows > 0 && cv_img->cols > 0);
  MU_ASSERT(cv_img->channels() == 1 || cv_img->channels() == 3 || cv_img->channels() == 4);
  MU_ASSERT(cv_img->channels() == this->channels());

  resize(cv_img->cols, cv_img->rows);
  if(channels() == 1) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      uint8_t* dst_p = &data_[0] + y * width;
      for(int x = 0; x < cv_img->cols; x++) dst_p[x] = cv_ptr[x];
    }
  } else if(channels() == 3) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      uint8_t* dst_p = &data_[0] + y * width * 3;
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < 3; c++) dst_p[x * 3 + c] = cv_ptr[x * 3 + c];
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      uint8_t* dst_p = &data_[0] + y * width * 4;
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < 4; c++) dst_p[x * 4 + c] = cv_ptr[x * 4 + c];
      }
    }
  } else {
    MU_FAIL("Invalid channels");
  }
}

void Image::to_cv_img(cv::Mat* cv_img) const {
  MU_ASSERT(cv_img);
  cv_img->create(height, width, CV_MAKETYPE(CV_8U, channels()));
  if(channels() == 1) {
    for(int y = 0; y < (int)height; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < (int)width; x++) {
        cv_ptr[x] = data_[y * width + x];
      }
    }
  } else if(channels() == 3) {
    for(int y = 0; y < (int)height; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      const uint8_t* src_p = &data_[0] + y * width * 3;
      for(int x = 0; x < (int)width; x++) {
        for(int c = 0; c < 3; c++) cv_ptr[x * 3 + c] = src_p[x * 3 + c];
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < (int)height; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      const uint8_t* src_p = &data_[0] + y * width * 4;
      for(int x = 0; x < (int)width; x++) {
        for(int c = 0; c < 4; c++) cv_ptr[x * 4 + c] = src_p[x * 4 + c];
      }
    }
  }
}

bool Image::copyto_rgba(Image* dst, const Vec2d& pmin) const {
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;
  MU_ASSERT(this->channels() == 4);
  MU_ASSERT(dst->channels() == 4);
  int cw = dst->width;
  int ch = dst->height;
  if(cw <= 0 || ch <= 0) return false;

  for(int y = 0; y < this->height; y++) {
    uint32_t* src_p = (uint32_t*)&data_[0] + y * width;
    int dy = pmin[1] + y;
    if(dy < 0 || dy >= ch) continue;
    uint32_t* dst_p = (uint32_t*)&dst->data_[0] + (dy)*cw;
    for(int x = 0; x < this->width; x++) {
      int dx = pmin[0] + x;
      if(dx < 0 || dx >= cw) continue;
      dst_p[dx] = src_p[x];
    }
  }
}

bool Image::copyto(Image* dst, const Vec2d& pmin) const {
  ScopeTimer st("Image::copyto");
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0 || dst->width <= 0 || dst->height <= 0) return false;
  if(this->channels() == 4 && dst->channels() == 4) return copyto_rgba(dst, pmin);
  int cw = dst->width;
  int ch = dst->height;
  if(cw <= 0 || ch <= 0) return false;

  for(int y = 0; y < this->height; y++) {
    int dy = pmin[1] + y;
    if(dy < 0 || dy >= ch) continue;
    for(int x = 0; x < this->width; x++) {
      int dx = pmin[0] + x;
      if(dx < 0 || dx >= cw) continue;
      auto rgba = this->rgba(x, y);
      dst->set_rgba(dx, dy, rgba);
    }
  }
  return true;
}

bool Image::copyto(Image* dst, const Vec2d& center, float scale, float angle) const {
  if(angle == 0 && scale == 1.0) return this->copyto(dst, center);
  // 角度をラジアンに変換
  float rad = angle * M_PI / 180.0f;
  float cx = center[0];
  float cy = center[1];

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

      auto rgba = this->rgba(src_x_int, src_y_int);
      dst->set_rgba(x, y, rgba);
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
  this->copyto(cmp->frame_final.get(), Vec2d(base_x, base_y));
}

Ref<Image> Image::Create(const char* name, const char* path, bool add_to_pj) {
  MU_ASSERT(name && path);
  auto img = std::make_shared<Image>();
  strncpy(img->name, name, sizeof(img->name));
  img->path = path;
  if(add_to_pj) {
    auto pj = Project::Get();
    MU_ASSERT(pj);
    pj->entities.push_back(img);
  }
  MU_FAIL("Not implemented (loading image using plugin...)");
}

Ref<Image> Image::Create(const char* name, int w, int h, ImageFormat format, bool add_to_pj) {
  MU_ASSERT(name && w > 0 && h > 0);
  auto img = std::make_shared<Image>();
  MU_ASSERT(img);
  strncpy(img->name, name, sizeof(img->name));
  img->fmt = format;
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

void cv_waitkey(int time) {
  cv::waitKey(time);
}

} // namespace mu
