#include <algorithm>
#include <cmath>
#include <movutl/asset/composition.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_vintage_filter.hpp>
#include <random>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

// ---------------- ビネット(周辺減光) ----------------
bool fn_proc_vignette(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Vignette::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 50.0f) / 100.0f;
  float radius   = std::clamp(cutil::get_or<float>(p, "radius", 70.0f) / 100.0f, 0.0f, 0.99f);
  if(strength <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx = w / 2.0f, cy = h / 2.0f;
  float max_dist = std::sqrt(cx * cx + cy * cy);
  if(max_dist <= 0.0f) return true;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dist   = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) / max_dist;
      float darken = std::clamp((dist - radius) / (1.0f - radius), 0.0f, 1.0f) * strength;
      Vec4b& px    = (*img)(x, y);
      for(int c = 0; c < 3; c++) px[c] = (uint8_t)(px[c] * (1.0f - darken));
    }
  }
  return true;
}
bool fn_init_vignette(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("radius", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("減光が始まる半径(%)");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 99.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("radius", 70.0f);
  return true;
}
FilterPluginTable f_vignette = {
  GUID(0x0001b), FilterDefault, cutil::Str("ビネット"), cutil::Str("ビネット"), 0, "0", nullptr, nullptr, fn_init_vignette, nullptr, fn_proc_vignette, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- フィルムグレイン ----------------
bool fn_proc_film_grain(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("FilmGrain::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 20.0f);
  if(strength <= 0.0f) return true;
  int frame = fpip->compo ? fpip->compo->frame.load() : 0;

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    std::mt19937 rng((unsigned)((i * 2654435761u) ^ (unsigned)(frame * 7919 + 12345)));
    std::uniform_real_distribution<float> dist(-strength, strength);
    float noise = dist(rng);
    for(int c = 0; c < 3; c++) px[i][c] = (uint8_t)std::clamp(px[i][c] + noise, 0.0f, 255.0f);
  }
  return true;
}
bool fn_init_film_grain(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 20.0f);
  return true;
}
FilterPluginTable f_film_grain = {
  GUID(0x0001c), FilterDefault, cutil::Str("フィルムグレイン"), cutil::Str("フィルムグレイン"), 0, "0", nullptr, nullptr, fn_init_film_grain, nullptr, fn_proc_film_grain, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 走査線 ----------------
bool fn_proc_scanline(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Scanline::fn_proc");

  float strength  = cutil::get_or<float>(p, "strength", 50.0f) / 100.0f;
  int line_width  = std::max(1, (int)cutil::get_or<float>(p, "line_width", 1.0f));
  if(strength <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    bool dark_line = ((y / line_width) % 2) == 0;
    if(!dark_line) continue;
    for(int x = 0; x < w; x++) {
      Vec4b& px = (*img)(x, y);
      for(int c = 0; c < 3; c++) px[c] = (uint8_t)(px[c] * (1.0f - strength));
    }
  }
  return true;
}
bool fn_init_scanline(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("line_width", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("線の太さ(px)");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 20.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("line_width", 1.0f);
  return true;
}
FilterPluginTable f_scanline = {
  GUID(0x0001d), FilterDefault, cutil::Str("走査線"), cutil::Str("走査線"), 0, "0", nullptr, nullptr, fn_init_scanline, nullptr, fn_proc_scanline, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- VHSノイズ(横シフト+ノイズ+走査線の複合) ----------------
bool fn_proc_vhs_noise(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("VhsNoise::fn_proc");

  float strength = cutil::get_or<float>(p, "strength", 50.0f) / 100.0f;
  if(strength <= 0.0f) return true;
  int frame = fpip->compo ? fpip->compo->frame.load() : 0;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    std::mt19937 row_rng((unsigned)(y * 9781 + frame * 131 + 7));
    std::uniform_real_distribution<float> shift_dist(-6.0f, 6.0f);
    std::uniform_real_distribution<float> noise_dist(-25.0f, 25.0f);
    int shift = (int)(shift_dist(row_rng) * strength);
    for(int x = 0; x < w; x++) {
      int sx      = std::clamp(x - shift, 0, w - 1);
      Vec4b pixel = src[(size_t)y * w + sx];
      float noise = noise_dist(row_rng);
      for(int c = 0; c < 3; c++) pixel[c] = (uint8_t)std::clamp(pixel[c] + noise * strength, 0.0f, 255.0f);
      if((y % 2) == 0) {
        for(int c = 0; c < 3; c++) pixel[c] = (uint8_t)(pixel[c] * (1.0f - 0.2f * strength));
      }
      (*img)(x, y) = pixel;
    }
  }
  return true;
}
bool fn_init_vhs_noise(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 50.0f);
  return true;
}
FilterPluginTable f_vhs_noise = {
  GUID(0x0001e), FilterDefault, cutil::Str("VHSノイズ"), cutil::Str("VHSノイズ"), 0, "0", nullptr, nullptr, fn_init_vhs_noise, nullptr, fn_proc_vhs_noise, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
