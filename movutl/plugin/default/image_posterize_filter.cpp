#include <algorithm>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_posterize_filter.hpp>
#include <opencv2/opencv.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- ポスタリゼーション ----------------
bool fn_proc_posterize(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Posterize::fn_proc");

  int levels = std::clamp((int)cutil::get_or<float>(p, "levels", 4.0f), 2, 32);

  uint8_t lut[256];
  for(int v = 0; v < 256; v++) lut[v] = (uint8_t)std::round(std::round(v / 255.0 * (levels - 1)) / (levels - 1) * 255.0);

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    px[i][0] = lut[px[i][0]];
    px[i][1] = lut[px[i][1]];
    px[i][2] = lut[px[i][2]];
  }
  return true;
}
bool fn_init_posterize(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("levels", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("階調数");
  props->fields.back().min_value  = 2.0f;
  props->fields.back().max_value  = 32.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("levels", 4.0f);
  return true;
}
FilterPluginTable f_posterize = {
  GUID(0x0001f), FilterDefault, cutil::Str("ポスタリゼーション"), cutil::Str("ポスタリゼーション"), 0, "0", nullptr, nullptr, fn_init_posterize, nullptr, fn_proc_posterize, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 2値化 ----------------
bool fn_proc_binarize(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Binarize::fn_proc");

  float threshold = cutil::get_or<float>(p, "threshold", 128.0f);
  Vec4b* px       = fpip->img->data();
  const size_t n  = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y      = 0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2];
    uint8_t out  = y >= threshold ? 255 : 0;
    px[i][0] = px[i][1] = px[i][2] = out;
  }
  return true;
}
bool fn_init_binarize(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("threshold", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("しきい値");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 255.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("threshold", 128.0f);
  return true;
}
FilterPluginTable f_binarize = {
  GUID(0x00020), FilterDefault, cutil::Str("2値化"), cutil::Str("2値化"), 0, "0", nullptr, nullptr, fn_init_binarize, nullptr, fn_proc_binarize, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- エンボス ----------------
bool fn_proc_emboss(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_UNUSED(p);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Emboss::fn_proc");

  cv::Mat src;
  fpip->img->to_cv_img(&src);
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
  cv::Mat kernel = (cv::Mat_<float>(3, 3) << -2, -1, 0, -1, 1, 1, 0, 1, 2);
  cv::Mat embossed;
  cv::filter2D(gray, embossed, -1, kernel, cv::Point(-1, -1), 128);
  cv::Mat out;
  cv::cvtColor(embossed, out, cv::COLOR_GRAY2BGRA);
  fpip->img->set_cv_img(&out);
  return true;
}
bool fn_init_emboss(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_UNUSED(props);
  MU_UNUSED(defaults);
  return true;
}
FilterPluginTable f_emboss = {
  GUID(0x00021), FilterDefault, cutil::Str("エンボス"), cutil::Str("エンボス"), 0, "0", nullptr, nullptr, fn_init_emboss, nullptr, fn_proc_emboss, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- ハーフトーン(ドット網点) ----------------
bool fn_proc_halftone(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Halftone::fn_proc");

  int dot_size = std::max(2, (int)cutil::get_or<float>(p, "dot_size", 8.0f));

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);
  img->fill_rgba(Vec4b(255, 255, 255, 255));

  int nby = (h + dot_size - 1) / dot_size;
#pragma omp parallel for schedule(static)
  for(int byi = 0; byi < nby; byi++) {
    int by = byi * dot_size;
    for(int bx = 0; bx < w; bx += dot_size) {
      float sum = 0.0f;
      int cnt   = 0;
      for(int y = by; y < std::min(by + dot_size, h); y++) {
        for(int x = bx; x < std::min(bx + dot_size, w); x++) {
          auto& s = src[(size_t)y * w + x];
          sum += 0.299f * s[0] + 0.587f * s[1] + 0.114f * s[2];
          cnt++;
        }
      }
      float avg = cnt > 0 ? sum / cnt : 255.0f;
      float r   = (255.0f - avg) / 255.0f * (dot_size / 2.0f);
      int cx = bx + dot_size / 2, cy = by + dot_size / 2;
      for(int y = std::max(0, by); y < std::min(h, by + dot_size); y++) {
        for(int x = std::max(0, bx); x < std::min(w, bx + dot_size); x++) {
          float dx = x - cx, dy = y - cy;
          if(dx * dx + dy * dy <= r * r) (*img)(x, y) = Vec4b(0, 0, 0, 255);
        }
      }
    }
  }
  return true;
}
bool fn_init_halftone(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("dot_size", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("ドットサイズ");
  props->fields.back().min_value  = 2.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("dot_size", 8.0f);
  return true;
}
FilterPluginTable f_halftone = {
  GUID(0x00022), FilterDefault, cutil::Str("ハーフトーン"), cutil::Str("ハーフトーン"), 0, "0", nullptr, nullptr, fn_init_halftone, nullptr, fn_proc_halftone, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
