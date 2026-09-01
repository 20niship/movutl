#include <movutl/core/profiler.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

bool fn_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);
  MOVUTL_ZONE_SCOPED_N("ColorCorrection::fn_proc");

  float brightness = cutil::get_or<float>(p, "brightness", 100.0f) / 100.0f;
  float contrast   = cutil::get_or<float>(p, "contrast", 100.0f) / 100.0f;
  if(brightness == 1.0f && contrast == 1.0f) return true; // 変化なしなら何もしない

  // 256階調のLUTを事前計算し、ピクセルごとの浮動小数点演算をテーブル参照に置き換える
  uint8_t lut[256];
  for(int v = 0; v < 256; v++) {
    float f = ((v - 127) * contrast + 127) * brightness;
    lut[v]  = (uint8_t)std::clamp(f, 0.0f, 255.0f);
  }

  Vec4b* px      = fpip->img->data();
  const size_t n = fpip->img->size();
  for(size_t i = 0; i < n; i++) {
    px[i][0] = lut[px[i][0]];
    px[i][1] = lut[px[i][1]];
    px[i][2] = lut[px[i][2]];
  }

  return true;
}
bool fn_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_ASSERT(props != nullptr);
  MU_ASSERT(defaults != nullptr);
  props->fields.push_back(cutil::PropInfo::Field("brightness", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("明るさ");
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
