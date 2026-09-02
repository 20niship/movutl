#include <algorithm>
#include <cmath>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_advcolor_filter.hpp>
#include <opencv2/opencv.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- カラーバランス(シャドウ/ハイライト個別調整) ----------------
bool fn_proc_color_balance(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ColorBalance::fn_proc");

  float shadow_r = cutil::get_or<float>(p, "shadow_r", 0.0f);
  float shadow_g = cutil::get_or<float>(p, "shadow_g", 0.0f);
  float shadow_b = cutil::get_or<float>(p, "shadow_b", 0.0f);
  float high_r   = cutil::get_or<float>(p, "highlight_r", 0.0f);
  float high_g   = cutil::get_or<float>(p, "highlight_g", 0.0f);
  float high_b   = cutil::get_or<float>(p, "highlight_b", 0.0f);
  if(shadow_r == 0 && shadow_g == 0 && shadow_b == 0 && high_r == 0 && high_g == 0 && high_b == 0) return true;

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y  = (0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2]) / 255.0f;
    float sw = 1.0f - y, hw = y;
    px[i][0] = (uint8_t)std::clamp(px[i][0] + shadow_r * sw + high_r * hw, 0.0f, 255.0f);
    px[i][1] = (uint8_t)std::clamp(px[i][1] + shadow_g * sw + high_g * hw, 0.0f, 255.0f);
    px[i][2] = (uint8_t)std::clamp(px[i][2] + shadow_b * sw + high_b * hw, 0.0f, 255.0f);
  }
  return true;
}
bool fn_init_color_balance(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  const char* names[6]  = {"shadow_r", "shadow_g", "shadow_b", "highlight_r", "highlight_g", "highlight_b"};
  const char* labels[6] = {"シャドウR", "シャドウG", "シャドウB", "ハイライトR", "ハイライトG", "ハイライトB"};
  for(int i = 0; i < 6; i++) {
    props->fields.push_back(cutil::PropInfo::Field(names[i], 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(labels[i]);
    props->fields.back().min_value  = -255.0f;
    props->fields.back().max_value  = 255.0f;
    props->fields.back().drag_speed = 1.0f;
    defaults->set<float>(names[i], 0.0f);
  }
  return true;
}
FilterPluginTable f_color_balance = {
  GUID(0x00027), FilterDefault, cutil::Str("カラーバランス"), cutil::Str("カラーバランス"), 0, "0", nullptr, nullptr, fn_init_color_balance, nullptr, fn_proc_color_balance, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- カラーLUT(RGB個別ガンマ) ----------------
bool fn_proc_color_lut(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ColorLut::fn_proc");

  float gamma_r = std::max(0.01f, cutil::get_or<float>(p, "gamma_r", 100.0f) / 100.0f);
  float gamma_g = std::max(0.01f, cutil::get_or<float>(p, "gamma_g", 100.0f) / 100.0f);
  float gamma_b = std::max(0.01f, cutil::get_or<float>(p, "gamma_b", 100.0f) / 100.0f);
  if(gamma_r == 1.0f && gamma_g == 1.0f && gamma_b == 1.0f) return true;

  uint8_t lut_r[256], lut_g[256], lut_b[256];
  for(int v = 0; v < 256; v++) {
    lut_r[v] = (uint8_t)std::clamp(std::pow(v / 255.0f, 1.0f / gamma_r) * 255.0f, 0.0f, 255.0f);
    lut_g[v] = (uint8_t)std::clamp(std::pow(v / 255.0f, 1.0f / gamma_g) * 255.0f, 0.0f, 255.0f);
    lut_b[v] = (uint8_t)std::clamp(std::pow(v / 255.0f, 1.0f / gamma_b) * 255.0f, 0.0f, 255.0f);
  }

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    px[i][0] = lut_r[px[i][0]];
    px[i][1] = lut_g[px[i][1]];
    px[i][2] = lut_b[px[i][2]];
  }
  return true;
}
bool fn_init_color_lut(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  const char* names[3]  = {"gamma_r", "gamma_g", "gamma_b"};
  const char* labels[3] = {"ガンマR", "ガンマG", "ガンマB"};
  for(int i = 0; i < 3; i++) {
    props->fields.push_back(cutil::PropInfo::Field(names[i], 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(labels[i]);
    props->fields.back().min_value  = 10.0f;
    props->fields.back().max_value  = 500.0f;
    props->fields.back().drag_speed = 1.0f;
    defaults->set<float>(names[i], 100.0f);
  }
  return true;
}
FilterPluginTable f_color_lut = {
  GUID(0x00028), FilterDefault, cutil::Str("カラーLUT"), cutil::Str("カラーLUT"), 0, "0", nullptr, nullptr, fn_init_color_lut, nullptr, fn_proc_color_lut, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- ソフトフォーカス ----------------
bool fn_proc_soft_focus(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("SoftFocus::fn_proc");

  float range   = cutil::get_or<float>(p, "range", 10.0f);
  float opacity = cutil::get_or<float>(p, "opacity", 50.0f) / 100.0f;
  if(range <= 0.0f || opacity <= 0.0f) return true;
  int ksize = ((int)range) * 2 + 1;

  cv::Mat src;
  fpip->img->to_cv_img(&src);
  cv::Mat blurred;
  cv::GaussianBlur(src, blurred, cv::Size(ksize, ksize), range / 2.0);
  cv::Mat out;
  cv::addWeighted(src, 1.0 - opacity, blurred, opacity, 0, out);
  fpip->img->set_cv_img(&out);
  return true;
}
bool fn_init_soft_focus(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 10.0f);

  props->fields.push_back(cutil::PropInfo::Field("opacity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("不透明度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("opacity", 50.0f);
  return true;
}
FilterPluginTable f_soft_focus = {
  GUID(0x00029), FilterDefault, cutil::Str("ソフトフォーカス"), cutil::Str("ソフトフォーカス"), 0, "0", nullptr, nullptr, fn_init_soft_focus, nullptr, fn_proc_soft_focus, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- インターレースシフト(奇数行を横シフト) ----------------
bool fn_proc_interlace_shift(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("InterlaceShift::fn_proc");

  int shift = (int)cutil::get_or<float>(p, "shift", 5.0f);
  if(shift == 0) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 1; y < h; y += 2) {
    for(int x = 0; x < w; x++) {
      int sx       = std::clamp(x - shift, 0, w - 1);
      (*img)(x, y) = src[(size_t)y * w + sx];
    }
  }
  return true;
}
bool fn_init_interlace_shift(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("shift", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("ずらし量(px)");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("shift", 5.0f);
  return true;
}
FilterPluginTable f_interlace_shift = {
  GUID(0x0002a), FilterDefault, cutil::Str("インターレースシフト"), cutil::Str("インターレースシフト"), 0, "0", nullptr, nullptr, fn_init_interlace_shift, nullptr, fn_proc_interlace_shift, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
