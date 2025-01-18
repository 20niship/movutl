#include <movutl/plugin/default/image_color_filter.hpp>

#define GUID(x) (0x0000000000000000 | x)

namespace mu {

bool fn_proc(void* fp, FilterInData* fpip, const Props& p) {
  return true;
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
  nullptr,       // fn_init
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

} // namespace mu
