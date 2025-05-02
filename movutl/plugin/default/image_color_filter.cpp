#include <movutl/plugin/default/image_color_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu::detail {

bool fn_proc(void* fp, FilterInData* fpip, const Props& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr);
  MU_ASSERT(fpip->img != nullptr);

  float brightness = p.get_or<float>("brightness", 0.0f) / 100.0f;
  float contrast = p.get_or<float>("contrast", 1.0f) / 100.0f;

  for(int i = 0; i < fpip->img->size(); i++) {
    Vec4b& pixel = (*fpip->img)[i];
    for(int j = 0; j < 3; j++) {
      float v = ((pixel[j] - 127) * contrast + 127) * brightness;
      pixel[j] = std::clamp(v, 0.0f, 255.0f);
    }
  }

  return true;
}
bool fn_init(void* fp, ExeData* editp, PropsInfo* props) {
  MU_ASSERT(props != nullptr);
  props->add_float_prop("brightness", "", "", 100, 0.0f, 1000.0f, 1.0f);
  props->set_last_prop_dispname("明るさ");
  props->add_float_prop("contrast", "", "", 100, 0.0f, 1000.0f, 1.0f);
  props->set_last_prop_dispname("コントラスト");
}

FilterPluginTable f_color_correction = {
  GUID(0x00001), // id
  FilterDefault, // flag
  "色調補正",    // name
  "色調補正",    // desc
  0,             // version
  "0",           // version str
  nullptr,       // fn_cutstom_wnd
  nullptr,       // fn_update_value
  fn_init,       // fn_init
  nullptr,       // fn_exit
  fn_proc,       // fn_proc
  nullptr,       // fn_update
  nullptr,       // func_is_saveframe
  nullptr,       // fn_project_load
  nullptr        // func_project_save
};

FilterPluginTable f_single_color;
FilterPluginTable f_color_shift;
FilterPluginTable f_gradient;
FilterPluginTable f_extend_color;
FilterPluginTable f_blur;
FilterPluginTable f_directional_blur;
FilterPluginTable f_radial_blur;

} // namespace mu::detail
