#include <algorithm>
#include <cmath>
#include <movutl/asset/composition.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_distortion_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- レンズ歪み(樽型/糸巻き型) ----------------
bool fn_proc_lens_distortion(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("LensDistortion::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 30.0f) / 100.0f;
  if(strength == 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx = w / 2.0f, cy = h / 2.0f;
  float maxr = std::sqrt(cx * cx + cy * cy);
  if(maxr <= 0.0f) return true;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dx = (x - cx) / maxr, dy = (y - cy) / maxr;
      float r      = std::sqrt(dx * dx + dy * dy);
      float factor = 1.0f + strength * r * r;
      int sx       = std::clamp((int)(cx + dx * maxr * factor), 0, w - 1);
      int sy       = std::clamp((int)(cy + dy * maxr * factor), 0, h - 1);
      (*img)(x, y) = src[(size_t)sy * w + sx];
    }
  }
  return true;
}
bool fn_init_lens_distortion(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ(+樽型/-糸巻き型)");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 30.0f);
  return true;
}
FilterPluginTable f_lens_distortion = {
  GUID(0x00023), FilterDefault, cutil::Str("レンズ歪み"), cutil::Str("レンズ歪み"), 0, "0", nullptr, nullptr, fn_init_lens_distortion, nullptr, fn_proc_lens_distortion, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 波紋 ----------------
bool fn_proc_ripple(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Ripple::fn_proc");

  float amplitude = cutil::get_or<float>(p, "amplitude", 10.0f);
  float frequency = cutil::get_or<float>(p, "frequency", 0.2f);
  float phase     = cutil::get_or<float>(p, "phase", 0.0f);
  if(amplitude == 0.0f) return true;
  int frame = fpip->compo ? fpip->compo->frame.load() : 0;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx = w / 2.0f, cy = h / 2.0f;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dx = x - cx, dy = y - cy;
      float dist   = std::sqrt(dx * dx + dy * dy);
      float wave   = std::sin(dist * frequency - frame * 0.2f + phase) * amplitude;
      float scale  = dist > 1.0f ? (dist + wave) / dist : 1.0f;
      int sx       = std::clamp((int)(cx + dx * scale), 0, w - 1);
      int sy       = std::clamp((int)(cy + dy * scale), 0, h - 1);
      (*img)(x, y) = src[(size_t)sy * w + sx];
    }
  }
  return true;
}
bool fn_init_ripple(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("amplitude", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("振幅");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("amplitude", 10.0f);

  props->fields.push_back(cutil::PropInfo::Field("frequency", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("周波数");
  props->fields.back().min_value  = 0.01f;
  props->fields.back().max_value  = 2.0f;
  props->fields.back().drag_speed = 0.01f;
  defaults->set<float>("frequency", 0.2f);

  props->fields.push_back(cutil::PropInfo::Field("phase", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("位相");
  props->fields.back().min_value  = -1000.0f;
  props->fields.back().max_value  = 1000.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("phase", 0.0f);
  return true;
}
FilterPluginTable f_ripple = {
  GUID(0x00024), FilterDefault, cutil::Str("波紋"), cutil::Str("波紋"), 0, "0", nullptr, nullptr, fn_init_ripple, nullptr, fn_proc_ripple, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 揺らぎ(水中揺らめき) ----------------
bool fn_proc_wave_distortion(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("WaveDistortion::fn_proc");

  float amplitude = cutil::get_or<float>(p, "amplitude", 5.0f);
  float frequency = cutil::get_or<float>(p, "frequency", 0.05f);
  if(amplitude == 0.0f) return true;
  int frame = fpip->compo ? fpip->compo->frame.load() : 0;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    float shift = std::sin(y * frequency + frame * 0.1f) * amplitude;
    for(int x = 0; x < w; x++) {
      int sx       = std::clamp((int)(x + shift), 0, w - 1);
      (*img)(x, y) = src[(size_t)y * w + sx];
    }
  }
  return true;
}
bool fn_init_wave_distortion(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("amplitude", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("振幅");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("amplitude", 5.0f);

  props->fields.push_back(cutil::PropInfo::Field("frequency", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("周波数");
  props->fields.back().min_value  = 0.01f;
  props->fields.back().max_value  = 1.0f;
  props->fields.back().drag_speed = 0.01f;
  defaults->set<float>("frequency", 0.05f);
  return true;
}
FilterPluginTable f_wave_distortion = {
  GUID(0x00025), FilterDefault, cutil::Str("揺らぎ"), cutil::Str("揺らぎ"), 0, "0", nullptr, nullptr, fn_init_wave_distortion, nullptr, fn_proc_wave_distortion, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 万華鏡 ----------------
bool fn_proc_kaleidoscope(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Kaleidoscope::fn_proc");

  int segments = std::max(2, (int)cutil::get_or<float>(p, "segments", 6.0f));

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx = w / 2.0f, cy = h / 2.0f;
  float seg_angle = 2.0f * (float)M_PI / segments;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dx = x - cx, dy = y - cy;
      float dist  = std::sqrt(dx * dx + dy * dy);
      float angle = std::atan2(dy, dx);
      float a     = std::fmod(angle, seg_angle);
      if(a < 0.0f) a += seg_angle;
      if(a > seg_angle / 2.0f) a = seg_angle - a;
      int sx       = std::clamp((int)(cx + std::cos(a) * dist), 0, w - 1);
      int sy       = std::clamp((int)(cy + std::sin(a) * dist), 0, h - 1);
      (*img)(x, y) = src[(size_t)sy * w + sx];
    }
  }
  return true;
}
bool fn_init_kaleidoscope(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("segments", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("分割数");
  props->fields.back().min_value  = 2.0f;
  props->fields.back().max_value  = 32.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("segments", 6.0f);
  return true;
}
FilterPluginTable f_kaleidoscope = {
  GUID(0x00026), FilterDefault, cutil::Str("万華鏡"), cutil::Str("万華鏡"), 0, "0", nullptr, nullptr, fn_init_kaleidoscope, nullptr, fn_proc_kaleidoscope, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
