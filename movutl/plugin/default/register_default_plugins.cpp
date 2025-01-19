#include <movutl/plugin/default/video_reader.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/app/app_impl.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

void register_default_plugins() {
  auto Main = AppMain::Get();
  Main->input_plugins.push_back(plg_video_reader);
}

void register_default_filters(){
  auto Main = AppMain::Get();
  Main->filters.push_back(f_color_correction);
}

} // namespace mu::deatail
