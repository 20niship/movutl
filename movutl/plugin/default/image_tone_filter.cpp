#include <algorithm>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_tone_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- 反転(ネガポジ) ----------------
bool fn_proc_invert(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Invert::fn_proc");

  bool invert_alpha = cutil::get_or<bool>(p, "invert_alpha", false);
  Vec4b* px         = fpip->img->data();
  const size_t n    = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    px[i][0] = 255 - px[i][0];
    px[i][1] = 255 - px[i][1];
    px[i][2] = 255 - px[i][2];
    if(invert_alpha) px[i][3] = 255 - px[i][3];
  }
  return true;
}
bool fn_init_invert(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("invert_alpha", 0, cutil::prop_info_of<bool>()));
  props->fields.back().set_label("透明度も反転");
  defaults->set<bool>("invert_alpha", false);
  return true;
}
FilterPluginTable f_invert = {
  GUID(0x0000f), FilterDefault, cutil::Str("反転"), cutil::Str("反転"), 0, "0", nullptr, nullptr, fn_init_invert, nullptr, fn_proc_invert, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- モノクロ ----------------
bool fn_proc_grayscale(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Grayscale::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 100.0f) / 100.0f;
  if(strength <= 0.0f) return true;
  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y = std::clamp(0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2], 0.0f, 255.0f);
    for(int c = 0; c < 3; c++) px[i][c] = (uint8_t)(px[i][c] * (1.0f - strength) + y * strength);
  }
  return true;
}
bool fn_init_grayscale(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 100.0f);
  return true;
}
FilterPluginTable f_grayscale = {
  GUID(0x00010), FilterDefault, cutil::Str("モノクロ"), cutil::Str("モノクロ"), 0, "0", nullptr, nullptr, fn_init_grayscale, nullptr, fn_proc_grayscale, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- セピア ----------------
bool fn_proc_sepia(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Sepia::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 100.0f) / 100.0f;
  if(strength <= 0.0f) return true;
  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y = 0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2];
    float r = std::clamp(y * 1.07f, 0.0f, 255.0f);
    float g = std::clamp(y * 0.74f, 0.0f, 255.0f);
    float b = std::clamp(y * 0.43f, 0.0f, 255.0f);
    px[i][0] = (uint8_t)(px[i][0] * (1.0f - strength) + r * strength);
    px[i][1] = (uint8_t)(px[i][1] * (1.0f - strength) + g * strength);
    px[i][2] = (uint8_t)(px[i][2] * (1.0f - strength) + b * strength);
  }
  return true;
}
bool fn_init_sepia(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 100.0f);
  return true;
}
FilterPluginTable f_sepia = {
  GUID(0x00011), FilterDefault, cutil::Str("セピア"), cutil::Str("セピア"), 0, "0", nullptr, nullptr, fn_init_sepia, nullptr, fn_proc_sepia, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
