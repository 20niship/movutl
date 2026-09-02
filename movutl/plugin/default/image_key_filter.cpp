#include <algorithm>
#include <cmath>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_key_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- クロマキー ----------------
bool fn_proc_chroma_key(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ChromaKey::fn_proc");

  Vec4b key       = cutil::get_or<Vec4b>(p, "key_color", Vec4b(0, 255, 0, 255));
  float threshold = cutil::get_or<float>(p, "threshold", 30.0f);
  float edge      = std::max(1.0f, cutil::get_or<float>(p, "edge", 10.0f)); // 閾値からフェードで透明になるまでの幅

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float dr        = (float)px[i][0] - key[0];
    float dg        = (float)px[i][1] - key[1];
    float db        = (float)px[i][2] - key[2];
    float dist      = std::sqrt(dr * dr + dg * dg + db * db);
    float alpha_mul = std::clamp((dist - threshold) / edge, 0.0f, 1.0f);
    px[i][3]        = (uint8_t)(px[i][3] * alpha_mul);
  }
  return true;
}
bool fn_init_chroma_key(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("key_color", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("抜き色");
  defaults->set<Vec4b>("key_color", Vec4b(0, 255, 0, 255));

  props->fields.push_back(cutil::PropInfo::Field("threshold", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("しきい値");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 442.0f; // sqrt(255^2*3)
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("threshold", 30.0f);

  props->fields.push_back(cutil::PropInfo::Field("edge", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("境界のぼかし幅");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("edge", 10.0f);
  return true;
}
FilterPluginTable f_chroma_key = {
  GUID(0x00009), FilterDefault, cutil::Str("クロマキー"), cutil::Str("クロマキー"), 0, "0", nullptr, nullptr, fn_init_chroma_key, nullptr, fn_proc_chroma_key, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- ルミナンスキー(輝度キー) ----------------
bool fn_proc_luminance_key(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("LuminanceKey::fn_proc");

  float threshold = cutil::get_or<float>(p, "threshold", 50.0f);
  float edge      = std::max(1.0f, cutil::get_or<float>(p, "edge", 10.0f));
  bool invert     = cutil::get_or<bool>(p, "invert", false); // false: 暗い部分を抜く, true: 明るい部分を抜く

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y         = 0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2];
    float diff       = invert ? (threshold - y) : (y - threshold);
    float alpha_mul = std::clamp(diff / edge, 0.0f, 1.0f);
    px[i][3]        = (uint8_t)(px[i][3] * alpha_mul);
  }
  return true;
}
bool fn_init_luminance_key(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("threshold", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("しきい値");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 255.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("threshold", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("edge", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("境界のぼかし幅");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("edge", 10.0f);

  props->fields.push_back(cutil::PropInfo::Field("invert", 0, cutil::prop_info_of<bool>()));
  props->fields.back().set_label("反転(明るい部分を抜く)");
  defaults->set<bool>("invert", false);
  return true;
}
FilterPluginTable f_luminance_key = {
  GUID(0x0000a), FilterDefault, cutil::Str("ルミナンスキー"), cutil::Str("ルミナンスキー"), 0, "0", nullptr, nullptr, fn_init_luminance_key, nullptr, fn_proc_luminance_key, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
