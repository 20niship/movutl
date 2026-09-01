#include <cutil/color.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

namespace {
// 明るさ(明度)/コントラストをRGB空間で256階調LUTにより適用する(色相/彩度に変化が無い場合の高速パス)
void apply_brightness_contrast_lut(Vec4b* px, size_t n, float brightness, float contrast) {
  if(brightness == 1.0f && contrast == 1.0f) return;
  uint8_t lut[256];
  for(int v = 0; v < 256; v++) {
    float f = ((v - 127) * contrast + 127) * brightness;
    lut[v]  = (uint8_t)std::clamp(f, 0.0f, 255.0f);
  }
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    px[i][0] = lut[px[i][0]];
    px[i][1] = lut[px[i][1]];
    px[i][2] = lut[px[i][2]];
  }
}
} // namespace

bool fn_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ColorCorrection::fn_proc");

  float brightness = cutil::get_or<float>(p, "brightness", 100.0f) / 100.0f;
  float contrast   = cutil::get_or<float>(p, "contrast", 100.0f) / 100.0f;
  float hue        = cutil::get_or<float>(p, "hue", 0.0f);          // 色相(度、-180〜180)
  float saturation = cutil::get_or<float>(p, "saturation", 100.0f); // 彩度(%、100=変化なし)

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();

  // 色相/彩度に変化が無ければRGB LUTのみの高速パスで済ませる
  if(hue == 0.0f && saturation == 100.0f) {
    apply_brightness_contrast_lut(px, n, brightness, contrast);
    return true;
  }

#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    cutil::Vector3b rgb{px[i][0], px[i][1], px[i][2]};
    auto hsv    = cutil::RGB2HSV<double>(rgb);
    double h    = std::fmod(hsv[0] + hue + 360.0, 360.0);
    double s    = std::clamp(hsv[1] * saturation / 100.0, 0.0, 100.0);
    auto out    = cutil::HSVtoRGB(cutil::_Vec<double, 3>{h, s, hsv[2]});
    px[i][0]    = out[0];
    px[i][1]    = out[1];
    px[i][2]    = out[2];
  }
  apply_brightness_contrast_lut(px, n, brightness, contrast);

  return true;
}
bool fn_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("hue", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("色相");
  props->fields.back().min_value  = -180.0f;
  props->fields.back().max_value  = 180.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("hue", 0.0f);

  props->fields.push_back(cutil::PropInfo::Field("saturation", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("彩度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("saturation", 100.0f);

  props->fields.push_back(cutil::PropInfo::Field("brightness", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("明度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 1000.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("brightness", 100.0f);

  props->fields.push_back(cutil::PropInfo::Field("contrast", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("コントラスト");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 1000.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("contrast", 100.0f);
  return true;
}

FilterPluginTable f_color_correction = {
  GUID(0x00001),          // id
  FilterDefault,          // flag
  cutil::Str("色調補正"), // name
  cutil::Str("色調補正"), // desc
  0,                      // version
  "0",                    // version str
  nullptr,                // fn_cutstom_wnd
  nullptr,                // fn_update_value
  fn_init,                // fn_init
  nullptr,                // fn_exit
  fn_proc,                // fn_proc
  nullptr,                // fn_update
  nullptr,                // func_is_saveframe
  nullptr,                // fn_project_load
  nullptr                 // func_project_save
};

FilterPluginTable f_single_color;
FilterPluginTable f_color_shift;
FilterPluginTable f_gradient;
FilterPluginTable f_extend_color;
FilterPluginTable f_blur;
FilterPluginTable f_directional_blur;
FilterPluginTable f_radial_blur;

} // namespace mu::detail
