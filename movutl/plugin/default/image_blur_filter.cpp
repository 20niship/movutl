#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_blur_filter.hpp>
#include <opencv2/opencv.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- ぼかし(ガウシアン) ----------------
bool fn_proc_blur(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Blur::fn_proc");

  float range = cutil::get_or<float>(p, "range", 5.0f);
  if(range <= 0.0f) return true;
  int ksize = ((int)range) * 2 + 1;

  cv::Mat mat;
  fpip->img->to_cv_img(&mat);
  cv::GaussianBlur(mat, mat, cv::Size(ksize, ksize), range / 2.0);
  fpip->img->set_cv_img(&mat);
  return true;
}
bool fn_init_blur(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 5.0f);
  return true;
}
FilterPluginTable f_blur = {
  GUID(0x00006), FilterDefault, cutil::Str("ぼかし"), cutil::Str("ぼかし"), 0, "0", nullptr, nullptr, fn_init_blur, nullptr, fn_proc_blur, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 方向ぼかし(モーションブラー) ----------------
bool fn_proc_directional_blur(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("DirectionalBlur::fn_proc");

  float range = cutil::get_or<float>(p, "range", 5.0f);
  float angle = cutil::get_or<float>(p, "angle", 0.0f) * (float)CV_PI / 180.0f;
  if(range <= 0.0f) return true;
  int ksize = ((int)range) * 2 + 1;

  cv::Mat kernel = cv::Mat::zeros(ksize, ksize, CV_32F);
  int c          = ksize / 2;
  float dx = std::cos(angle), dy = std::sin(angle);
  int count = 0;
  for(int i = -c; i <= c; i++) {
    int x = c + (int)std::round(dx * i);
    int y = c + (int)std::round(dy * i);
    if(x >= 0 && x < ksize && y >= 0 && y < ksize) {
      kernel.at<float>(y, x) += 1.0f;
      count++;
    }
  }
  if(count == 0) return true;
  kernel /= (float)count;

  cv::Mat mat;
  fpip->img->to_cv_img(&mat);
  cv::filter2D(mat, mat, -1, kernel);
  fpip->img->set_cv_img(&mat);
  return true;
}
bool fn_init_directional_blur(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 5.0f);

  props->fields.push_back(cutil::PropInfo::Field("angle", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("角度");
  props->fields.back().min_value  = -180.0f;
  props->fields.back().max_value  = 180.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("angle", 0.0f);
  return true;
}
FilterPluginTable f_directional_blur = {
  GUID(0x00007), FilterDefault, cutil::Str("方向ぼかし"), cutil::Str("方向ぼかし"), 0, "0", nullptr, nullptr, fn_init_directional_blur, nullptr, fn_proc_directional_blur, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 放射ぼかし(ズームブラー) ----------------
bool fn_proc_radial_blur(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("RadialBlur::fn_proc");

  float range = cutil::get_or<float>(p, "range", 5.0f) / 100.0f;
  if(range <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx = w / 2.0f, cy = h / 2.0f;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);
  constexpr int kSamples = 8;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float sum[4] = {0, 0, 0, 0};
      for(int s = 0; s < kSamples; s++) {
        float t  = 1.0f - range * (float)s / kSamples;
        int sx   = (int)std::clamp(cx + (x - cx) * t, 0.0f, (float)(w - 1));
        int sy   = (int)std::clamp(cy + (y - cy) * t, 0.0f, (float)(h - 1));
        auto& sp = src[(size_t)sy * w + sx];
        for(int c = 0; c < 4; c++) sum[c] += sp[c];
      }
      auto& dst = (*img)(x, y);
      for(int c = 0; c < 4; c++) dst[c] = (uint8_t)(sum[c] / kSamples);
    }
  }
  return true;
}
bool fn_init_radial_blur(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 5.0f);
  return true;
}
FilterPluginTable f_radial_blur = {
  GUID(0x00008), FilterDefault, cutil::Str("放射ぼかし"), cutil::Str("放射ぼかし"), 0, "0", nullptr, nullptr, fn_init_radial_blur, nullptr, fn_proc_radial_blur, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
