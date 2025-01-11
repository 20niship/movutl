#include <opencv2/opencv.hpp>
//
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>

namespace mu {

void Image::set_cv_img(const cv::Mat* cv_img) {
  MU_ASSERT(cv_img);
  if(cv_img->channels() == 1)
    this->_format = Format::FormatGRAYSCALE;
  else if(cv_img->channels() == 3)
    this->_format = Format::FormatRGB;
  else if(cv_img->channels() == 4)
    this->_format = Format::FormatRGBA;
  else
    MU_ASSERT(false);

  resize(cv_img->cols, cv_img->rows);
  if(channels() == 1) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        data_[y * width_ + x] = cv_ptr[x];
      }
    }
  } else if(channels() == 3) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < channels(); c++) {
          data_[(y * width_ + x) * channels() + c] = cv_ptr[x * channels() + c];
        }
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < cv_img->rows; y++) {
      const uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < cv_img->cols; x++) {
        for(int c = 0; c < channels(); c++) {
          data_[(y * width_ + x) * channels() + c] = cv_ptr[x * channels() + c];
        }
      }
    }
  } else {
    MU_FAIL("Invalid channels");
  }
}

void Image::to_cv_img(cv::Mat* cv_img) const {
  MU_ASSERT(cv_img);
  cv_img->create(height_, width_, CV_MAKETYPE(CV_8U, channels()));
  if(channels() == 1) {
    for(int y = 0; y < (int)height_; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < (int)width_; x++) {
        cv_ptr[x] = data_[y * width_ + x];
      }
    }
  } else if(channels() == 3) {
    for(int y = 0; y < (int)height_; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < (int)width_; x++) {
        for(int c = 0; c < channels(); c++) {
          cv_ptr[x * channels() + c] = data_[(y * width_ + x) * channels() + c];
        }
      }
    }
  } else if(channels() == 4) {
    for(int y = 0; y < (int)height_; y++) {
      uint8_t* cv_ptr = cv_img->ptr<uint8_t>(y);
      for(int x = 0; x < (int)width_; x++) {
        for(int c = 0; c < channels(); c++) {
          cv_ptr[x * channels() + c] = data_[(y * width_ + x) * channels() + c];
        }
      }
    }
  }
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
