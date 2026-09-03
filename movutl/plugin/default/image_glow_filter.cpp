#include <algorithm>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_glow_filter.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- 発光: 明るい部分だけ抽出してぼかし加算 ----------------
bool fn_proc_glow(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Glow::fn_proc");

  float threshold = cutil::get_or<float>(p, "threshold", 200.0f);
  float range     = cutil::get_or<float>(p, "range", 10.0f);
  float intensity = cutil::get_or<float>(p, "intensity", 100.0f) / 100.0f;
  if(range <= 0.0f || intensity <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  Vec4b* px      = img->data();
  const size_t n = img->size();

  std::vector<Vec4b> bright(n);
  for(size_t i = 0; i < n; i++) {
    float y   = 0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2];
    bright[i] = y >= threshold ? px[i] : Vec4b(0, 0, 0, 0);
  }

  cv::Mat bmat(h, w, CV_8UC4, bright.data());
  int ksize = ((int)range) * 2 + 1;
  cv::GaussianBlur(bmat, bmat, cv::Size(ksize, ksize), range / 2.0);

#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    for(int c = 0; c < 3; c++) px[i][c] = (uint8_t)std::clamp(px[i][c] + bright[i][c] * intensity, 0.0f, 255.0f);
  }
  return true;
}
bool fn_init_glow(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("threshold", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("しきい値");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 255.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("threshold", 200.0f);

  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 10.0f);

  props->fields.push_back(cutil::PropInfo::Field("intensity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 500.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("intensity", 100.0f);
  return true;
}
FilterPluginTable f_glow = {
  GUID(0x0000b), FilterDefault, cutil::Str("発光"), cutil::Str("発光"), 0, "0", nullptr, nullptr, fn_init_glow, nullptr, fn_proc_glow, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- グロー: 画像全体をぼかしてscreen合成する柔らかいブルーム ----------------
bool fn_proc_bloom(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Bloom::fn_proc");

  float range     = cutil::get_or<float>(p, "range", 10.0f);
  float intensity = cutil::get_or<float>(p, "intensity", 50.0f) / 100.0f;
  if(range <= 0.0f || intensity <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  Vec4b* px      = img->data();
  const size_t n = img->size();

  std::vector<Vec4b> blurred(px, px + n);
  cv::Mat bmat(h, w, CV_8UC4, blurred.data());
  int ksize = ((int)range) * 2 + 1;
  cv::GaussianBlur(bmat, bmat, cv::Size(ksize, ksize), range / 2.0);

#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    for(int c = 0; c < 3; c++) {
      float a      = px[i][c];
      float b      = blurred[i][c] * intensity;
      float screen = 255.0f - (255.0f - a) * (255.0f - b) / 255.0f;
      px[i][c]     = (uint8_t)std::clamp(screen, 0.0f, 255.0f);
    }
  }
  return true;
}
bool fn_init_bloom(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("range", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("範囲");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("range", 10.0f);

  props->fields.push_back(cutil::PropInfo::Field("intensity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("intensity", 50.0f);
  return true;
}
FilterPluginTable f_bloom = {
  GUID(0x0000c), FilterDefault, cutil::Str("グロー"), cutil::Str("グロー"), 0, "0", nullptr, nullptr, fn_init_bloom, nullptr, fn_proc_bloom, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
