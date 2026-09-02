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
    auto hsv = cutil::RGB2HSV<double>(rgb);
    double h = std::fmod(hsv[0] + hue + 360.0, 360.0);
    double s = std::clamp(hsv[1] * saturation / 100.0, 0.0, 100.0);
    auto out = cutil::HSVtoRGB(cutil::_Vec<double, 3>{h, s, hsv[2]});
    px[i][0] = out[0];
    px[i][1] = out[1];
    px[i][2] = out[2];
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

// ---------------- 単色化 ----------------
bool fn_proc_single_color(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("SingleColor::fn_proc");

  Vec4b color    = cutil::get_or<Vec4b>(p, "color", Vec4b(255, 255, 255, 255));
  float strength = cutil::get_or<float>(p, "strength", 100.0f) / 100.0f;
  if(strength <= 0.0f) return true;

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
#pragma omp parallel for schedule(static)
  for(long i = 0; i < (long)n; i++) {
    float y = (0.299f * px[i][0] + 0.587f * px[i][1] + 0.114f * px[i][2]) / 255.0f;
    for(int c = 0; c < 3; c++) {
      float v  = color[c] * y;
      px[i][c] = (uint8_t)std::clamp(px[i][c] * (1.0f - strength) + v * strength, 0.0f, 255.0f);
    }
  }
  return true;
}
bool fn_init_single_color(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("color", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("色");
  defaults->set<Vec4b>("color", Vec4b(255, 255, 255, 255));

  props->fields.push_back(cutil::PropInfo::Field("strength", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("強さ");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("strength", 100.0f);
  return true;
}
FilterPluginTable f_single_color = {
  GUID(0x00002), FilterDefault, cutil::Str("単色化"), cutil::Str("単色化"), 0, "0", nullptr, nullptr, fn_init_single_color, nullptr, fn_proc_single_color, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 色ずらし ----------------
bool fn_proc_color_shift(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ColorShift::fn_proc");

  int sx = (int)cutil::get_or<float>(p, "shift_x", 5.0f);
  int sy = (int)cutil::get_or<float>(p, "shift_y", 0.0f);
  if(sx == 0 && sy == 0) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  std::vector<Vec4b> src(img->data(), img->data() + (size_t)w * h);

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      // Rチャンネルは-方向、Bチャンネルは+方向にずらして色収差風の見た目にする
      int rx = std::clamp(x - sx, 0, w - 1), ry = std::clamp(y - sy, 0, h - 1);
      int bx = std::clamp(x + sx, 0, w - 1), by = std::clamp(y + sy, 0, h - 1);
      Vec4b& dst = (*img)(x, y);
      dst[0]     = src[(size_t)ry * w + rx][0];
      dst[2]     = src[(size_t)by * w + bx][2];
    }
  }
  return true;
}
bool fn_init_color_shift(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("shift_x", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("X");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("shift_x", 5.0f);

  props->fields.push_back(cutil::PropInfo::Field("shift_y", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("Y");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("shift_y", 0.0f);
  return true;
}
FilterPluginTable f_color_shift = {
  GUID(0x00003), FilterDefault, cutil::Str("色ずらし"), cutil::Str("色ずらし"), 0, "0", nullptr, nullptr, fn_init_color_shift, nullptr, fn_proc_color_shift, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- グラデーション ----------------
bool fn_proc_gradient(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("Gradient::fn_proc");

  Vec4b color1  = cutil::get_or<Vec4b>(p, "color1", Vec4b(0, 0, 0, 255));
  Vec4b color2  = cutil::get_or<Vec4b>(p, "color2", Vec4b(255, 255, 255, 255));
  float angle   = cutil::get_or<float>(p, "angle", 0.0f) * (float)M_PI / 180.0f;
  float opacity = cutil::get_or<float>(p, "opacity", 100.0f) / 100.0f;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float dx = std::cos(angle), dy = std::sin(angle);
  float diag = std::sqrt((float)(w * w + h * h));
  if(diag <= 0.0f) return true;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float t   = ((x - w / 2.0f) * dx + (y - h / 2.0f) * dy) / diag + 0.5f;
      t         = std::clamp(t, 0.0f, 1.0f);
      Vec4b& px = (*img)(x, y);
      float a   = (color1[3] * (1.0f - t) + color2[3] * t) / 255.0f * opacity;
      for(int c = 0; c < 3; c++) {
        float g = color1[c] * (1.0f - t) + color2[c] * t;
        px[c]   = (uint8_t)std::clamp(px[c] * (1.0f - a) + g * a, 0.0f, 255.0f);
      }
    }
  }
  return true;
}
bool fn_init_gradient(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("color1", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("開始色");
  defaults->set<Vec4b>("color1", Vec4b(0, 0, 0, 255));

  props->fields.push_back(cutil::PropInfo::Field("color2", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("終了色");
  defaults->set<Vec4b>("color2", Vec4b(255, 255, 255, 255));

  props->fields.push_back(cutil::PropInfo::Field("angle", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("角度");
  props->fields.back().min_value  = -180.0f;
  props->fields.back().max_value  = 180.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("angle", 0.0f);

  props->fields.push_back(cutil::PropInfo::Field("opacity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("不透明度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("opacity", 100.0f);
  return true;
}
FilterPluginTable f_gradient = {
  GUID(0x00004), FilterDefault, cutil::Str("グラデーション"), cutil::Str("グラデーション"), 0, "0", nullptr, nullptr, fn_init_gradient, nullptr, fn_proc_gradient, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 拡張色調補正(RGB個別オフセット) ----------------
bool fn_proc_extend_color(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ExtendColor::fn_proc");

  float r_add = cutil::get_or<float>(p, "r", 0.0f);
  float g_add = cutil::get_or<float>(p, "g", 0.0f);
  float b_add = cutil::get_or<float>(p, "b", 0.0f);
  if(r_add == 0.0f && g_add == 0.0f && b_add == 0.0f) return true;

  uint8_t lut_r[256], lut_g[256], lut_b[256];
  for(int v = 0; v < 256; v++) {
    lut_r[v] = (uint8_t)std::clamp(v + r_add, 0.0f, 255.0f);
    lut_g[v] = (uint8_t)std::clamp(v + g_add, 0.0f, 255.0f);
    lut_b[v] = (uint8_t)std::clamp(v + b_add, 0.0f, 255.0f);
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
bool fn_init_extend_color(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  const char* names[3]  = {"r", "g", "b"};
  const char* labels[3] = {"R", "G", "B"};
  for(int i = 0; i < 3; i++) {
    props->fields.push_back(cutil::PropInfo::Field(names[i], 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(labels[i]);
    props->fields.back().min_value  = -255.0f;
    props->fields.back().max_value  = 255.0f;
    props->fields.back().drag_speed = 1.0f;
    defaults->set<float>(names[i], 0.0f);
  }
  return true;
}
FilterPluginTable f_extend_color = {
  GUID(0x00005), FilterDefault, cutil::Str("拡張色調補正"), cutil::Str("拡張色調補正"), 0, "0", nullptr, nullptr, fn_init_extend_color, nullptr, fn_proc_extend_color, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
