#include <opencv2/opencv.hpp>
//
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>

namespace mu {

void Image::set_cv_img(const cv::Mat* cv_img) {
  MU_ASSERT(cv_img);
  if(cv_img->channels() == 1)
    this->fmt = Format::FormatGRAYSCALE;
  else if(cv_img->channels() == 3)
    this->fmt = Format::FormatRGB;
  else if(cv_img->channels() == 4)
    this->fmt = Format::FormatRGBA;
  else
    MU_ASSERT(false);

  resize(cv_img->cols, cv_img->rows);
  if(channels() == 1) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        data_[y * width + x] = cv_ptr[x];
      }
    }
  } else if(channels() == 3) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < channels(); c++) {
          data_[(y * width + x) * channels() + c] = cv_ptr[x * channels() + c];
        }
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < channels(); c++) {
          data_[(y * width + x) * channels() + c] = cv_ptr[x * channels() + c];
        }
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
      for(int x = 0; x < (int)width; x++) {
        for(int c = 0; c < channels(); c++) {
          cv_ptr[x * channels() + c] = data_[(y * width + x) * channels() + c];
        }
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < (int)height; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < (int)width; x++) {
        for(int c = 0; c < channels(); c++) {
          cv_ptr[x * channels() + c] = data_[(y * width + x) * channels() + c];
        }
      }
    }
  }
}

void Image::copyto(Image* dst, const Vec2d& pmin) const {
  MU_ASSERT(dst);
  if(this->width <= 0 || this->height <= 0) return false;
  int cw = dst->width;
  int ch = dst->height;
  if(cw <= 0 || ch <= 0) return false;

  for(int y = 0; y < this->height; y++) {
    for(int x = 0; x < this->width; x++) {
      if(x >= cw || y >= ch) continue;
      int dx = pmin[0] + x;
      int dy = pmin[1] + y;
      if(dx < 0 || dx >= cw || dy < 0 || dy >= ch) continue;
      auto rgba = this->rgba(x, y);
      dst->set_rgba(dx, dy, rgba);
    }
  }
}
void Image::copyto(Image* dst, const Vec2d& pmin, float scale, const Vec2d& offset) const {}
void Image::copyto(Image* dst, const Vec2d& pmin, const Vec2d& pmax) const {}
void Image::copyto(Image* dst, const Vec2d& center, float scale, float angle) const {}

bool Image::render(Composition* cmp) {
  MU_ASSERT(cmp);
  MU_ASSERT(cmp->frame_final);
  if(this->width <= 0 || this->height <= 0) return false;
  int cw = cmp->size[0];
  int ch = cmp->size[1];
  if(cw <= 0 || ch <= 0) return false;

  int base_x = this->width / 2 + trk.anchor_x - cw / 2;
  int base_y = this->height / 2 + trk.anchor_y - ch / 2;
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

Ref<Image> Image::Create(const char* name, int w, int h, Format format, bool add_to_pj) {
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

void Image::imshow() const {
  cv::Mat cv_img;
  to_cv_img(&cv_img);
  cv::imshow("Image", cv_img);
  cv_waitkey();
}

void cv_waitkey(int time) {
  cv::waitKey(time);
}

} // namespace mu
