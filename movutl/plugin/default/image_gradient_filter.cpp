#include <algorithm>
#include <cmath>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_gradient_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

namespace {
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
} // namespace

// ---------------- 4色グラデーション ----------------
bool fn_proc_four_color_gradient(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("FourColorGradient::fn_proc");

  Vec4b tl      = cutil::get_or<Vec4b>(p, "color_tl", Vec4b(255, 0, 0, 255));
  Vec4b tr      = cutil::get_or<Vec4b>(p, "color_tr", Vec4b(0, 255, 0, 255));
  Vec4b bl      = cutil::get_or<Vec4b>(p, "color_bl", Vec4b(0, 0, 255, 255));
  Vec4b br      = cutil::get_or<Vec4b>(p, "color_br", Vec4b(255, 255, 0, 255));
  float opacity = cutil::get_or<float>(p, "opacity", 100.0f) / 100.0f;
  if(opacity <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    float v = h > 1 ? y / (float)(h - 1) : 0.0f;
    for(int x = 0; x < w; x++) {
      float u   = w > 1 ? x / (float)(w - 1) : 0.0f;
      Vec4b& px = (*img)(x, y);
      float a   = opacity * lerpf(lerpf(tl[3], tr[3], u), lerpf(bl[3], br[3], u), v) / 255.0f;
      for(int c = 0; c < 3; c++) {
        float g  = lerpf(lerpf(tl[c], tr[c], u), lerpf(bl[c], br[c], u), v);
        px[c]    = (uint8_t)std::clamp(px[c] * (1.0f - a) + g * a, 0.0f, 255.0f);
      }
    }
  }
  return true;
}
bool fn_init_four_color_gradient(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  const char* names[4]  = {"color_tl", "color_tr", "color_bl", "color_br"};
  const char* labels[4] = {"左上", "右上", "左下", "右下"};
  Vec4b init_[4]        = {Vec4b(255, 0, 0, 255), Vec4b(0, 255, 0, 255), Vec4b(0, 0, 255, 255), Vec4b(255, 255, 0, 255)};
  for(int i = 0; i < 4; i++) {
    props->fields.push_back(cutil::PropInfo::Field(names[i], 0, cutil::prop_info_of<Vec4b>()));
    props->fields.back().set_label(labels[i]);
    defaults->set<Vec4b>(names[i], init_[i]);
  }
  props->fields.push_back(cutil::PropInfo::Field("opacity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("不透明度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("opacity", 100.0f);
  return true;
}
FilterPluginTable f_four_color_gradient = {
  GUID(0x00017), FilterDefault, cutil::Str("4色グラデーション"), cutil::Str("4色グラデーション"), 0, "0", nullptr, nullptr, fn_init_four_color_gradient, nullptr, fn_proc_four_color_gradient, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 放射グラデーション ----------------
bool fn_proc_radial_gradient(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("RadialGradient::fn_proc");

  Vec4b color1  = cutil::get_or<Vec4b>(p, "color1", Vec4b(255, 255, 255, 255));
  Vec4b color2  = cutil::get_or<Vec4b>(p, "color2", Vec4b(0, 0, 0, 255));
  float radius  = cutil::get_or<float>(p, "radius", 50.0f);
  float cx_pct  = cutil::get_or<float>(p, "center_x", 50.0f);
  float cy_pct  = cutil::get_or<float>(p, "center_y", 50.0f);
  float opacity = cutil::get_or<float>(p, "opacity", 100.0f) / 100.0f;
  if(opacity <= 0.0f || radius <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx  = w * cx_pct / 100.0f, cy = h * cy_pct / 100.0f;
  float rpx = std::max(w, h) * radius / 100.0f;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dx = x - cx, dy = y - cy;
      float t   = std::clamp(std::sqrt(dx * dx + dy * dy) / rpx, 0.0f, 1.0f);
      Vec4b& px = (*img)(x, y);
      float a   = opacity * lerpf(color1[3], color2[3], t) / 255.0f;
      for(int c = 0; c < 3; c++) {
        float g = lerpf(color1[c], color2[c], t);
        px[c]   = (uint8_t)std::clamp(px[c] * (1.0f - a) + g * a, 0.0f, 255.0f);
      }
    }
  }
  return true;
}
bool fn_init_radial_gradient(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("color1", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("中心色");
  defaults->set<Vec4b>("color1", Vec4b(255, 255, 255, 255));

  props->fields.push_back(cutil::PropInfo::Field("color2", 0, cutil::prop_info_of<Vec4b>()));
  props->fields.back().set_label("外側色");
  defaults->set<Vec4b>("color2", Vec4b(0, 0, 0, 255));

  props->fields.push_back(cutil::PropInfo::Field("radius", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("半径");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("radius", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("center_x", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("中心X(%)");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("center_x", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("center_y", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("中心Y(%)");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("center_y", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("opacity", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("不透明度");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("opacity", 100.0f);
  return true;
}
FilterPluginTable f_radial_gradient = {
  GUID(0x00018), FilterDefault, cutil::Str("放射グラデーション"), cutil::Str("放射グラデーション"), 0, "0", nullptr, nullptr, fn_init_radial_gradient, nullptr, fn_proc_radial_gradient, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 斜めクリッピング ----------------
bool fn_proc_diagonal_clip(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("DiagonalClip::fn_proc");

  float angle  = cutil::get_or<float>(p, "angle", 45.0f) * (float)M_PI / 180.0f;
  float offset = cutil::get_or<float>(p, "offset", 0.0f); // -100〜100(%): 直線を中心からどれだけずらすか
  float edge   = std::max(0.5f, cutil::get_or<float>(p, "edge", 1.0f));
  bool invert  = cutil::get_or<bool>(p, "invert", false);

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float nx = std::cos(angle), ny = std::sin(angle); // 直線の法線方向
  float diag = std::sqrt((float)(w * w + h * h));
  float off  = offset / 100.0f * diag / 2.0f;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float d = (x - w / 2.0f) * nx + (y - h / 2.0f) * ny - off;
      if(invert) d = -d;
      float alpha_mul = std::clamp(0.5f - d / edge, 0.0f, 1.0f);
      Vec4b& px       = (*img)(x, y);
      px[3]           = (uint8_t)(px[3] * alpha_mul);
    }
  }
  return true;
}
bool fn_init_diagonal_clip(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("angle", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("角度");
  props->fields.back().min_value  = -180.0f;
  props->fields.back().max_value  = 180.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("angle", 45.0f);

  props->fields.push_back(cutil::PropInfo::Field("offset", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("オフセット");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("offset", 0.0f);

  props->fields.push_back(cutil::PropInfo::Field("edge", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("境界のぼかし幅");
  props->fields.back().min_value  = 0.5f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("edge", 1.0f);

  props->fields.push_back(cutil::PropInfo::Field("invert", 0, cutil::prop_info_of<bool>()));
  props->fields.back().set_label("反転");
  defaults->set<bool>("invert", false);
  return true;
}
FilterPluginTable f_diagonal_clip = {
  GUID(0x00019), FilterDefault, cutil::Str("斜めクリッピング"), cutil::Str("斜めクリッピング"), 0, "0", nullptr, nullptr, fn_init_diagonal_clip, nullptr, fn_proc_diagonal_clip, nullptr, nullptr, nullptr, nullptr,
};

// ---------------- 円形クリッピング ----------------
bool fn_proc_circle_clip(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("CircleClip::fn_proc");

  float radius = cutil::get_or<float>(p, "radius", 50.0f);
  float cx_pct = cutil::get_or<float>(p, "center_x", 50.0f);
  float cy_pct = cutil::get_or<float>(p, "center_y", 50.0f);
  float edge   = std::max(0.5f, cutil::get_or<float>(p, "edge", 1.0f));
  bool invert  = cutil::get_or<bool>(p, "invert", false);
  if(radius <= 0.0f) return true;

  Image* img = fpip->img;
  int w = (int)img->width, h = (int)img->height;
  float cx  = w * cx_pct / 100.0f, cy = h * cy_pct / 100.0f;
  float rpx = std::max(w, h) * radius / 100.0f;

#pragma omp parallel for schedule(static)
  for(int y = 0; y < h; y++) {
    for(int x = 0; x < w; x++) {
      float dx = x - cx, dy = y - cy;
      float dist = std::sqrt(dx * dx + dy * dy);
      float d    = invert ? (dist - rpx) : (rpx - dist);
      float alpha_mul = std::clamp(0.5f + d / edge, 0.0f, 1.0f);
      Vec4b& px       = (*img)(x, y);
      px[3]           = (uint8_t)(px[3] * alpha_mul);
    }
  }
  return true;
}
bool fn_init_circle_clip(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("radius", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("半径(%)");
  props->fields.back().min_value  = 1.0f;
  props->fields.back().max_value  = 200.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("radius", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("center_x", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("中心X(%)");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("center_x", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("center_y", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("中心Y(%)");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("center_y", 50.0f);

  props->fields.push_back(cutil::PropInfo::Field("edge", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("境界のぼかし幅");
  props->fields.back().min_value  = 0.5f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("edge", 1.0f);

  props->fields.push_back(cutil::PropInfo::Field("invert", 0, cutil::prop_info_of<bool>()));
  props->fields.back().set_label("反転(内側を消す)");
  defaults->set<bool>("invert", false);
  return true;
}
FilterPluginTable f_circle_clip = {
  GUID(0x0001a), FilterDefault, cutil::Str("円形クリッピング"), cutil::Str("円形クリッピング"), 0, "0", nullptr, nullptr, fn_init_circle_clip, nullptr, fn_proc_circle_clip, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
