#include <movutl/app/app_impl.hpp>
#include <movutl/plugin/default/audio_filters.hpp>
#include <movutl/plugin/default/audio_volume_filter.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

void register_default_plugins() {
  // video_reader は plugins/video_reader/*.mso として外部プラグイン経由で登録される
}

void register_default_filters() {
  auto Main = AppMain::Get();
  Main->filters.push_back(f_color_correction);
  Main->filters.push_back(f_audio_volume);
  Main->filters.push_back(f_audio_pan);
  Main->filters.push_back(f_audio_eq);
  Main->filters.push_back(f_audio_reverb);
  Main->filters.push_back(f_audio_echo);
}

} // namespace mu::detail
