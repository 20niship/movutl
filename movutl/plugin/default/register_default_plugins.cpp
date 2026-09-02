#include <movutl/app/app_impl.hpp>
#include <movutl/plugin/default/audio_filters.hpp>
#include <movutl/plugin/default/audio_volume_filter.hpp>
#include <movutl/plugin/default/image_blur_filter.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/default/image_effect_filter.hpp>
#include <movutl/plugin/default/image_glow_filter.hpp>
#include <movutl/plugin/default/image_key_filter.hpp>
#include <movutl/plugin/default/image_outline_filter.hpp>
#include <movutl/plugin/default/image_tone_filter.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

void register_default_plugins() {
  // video_reader は plugins/video_reader/*.mso として外部プラグイン経由で登録される
}

void register_default_filters() {
  auto Main = AppMain::Get();
  Main->filters.push_back(f_color_correction);
  Main->filters.push_back(f_single_color);
  Main->filters.push_back(f_color_shift);
  Main->filters.push_back(f_gradient);
  Main->filters.push_back(f_extend_color);
  Main->filters.push_back(f_blur);
  Main->filters.push_back(f_directional_blur);
  Main->filters.push_back(f_radial_blur);
  Main->filters.push_back(f_chroma_key);
  Main->filters.push_back(f_luminance_key);
  Main->filters.push_back(f_glow);
  Main->filters.push_back(f_bloom);
  Main->filters.push_back(f_outline);
  Main->filters.push_back(f_clipping);
  Main->filters.push_back(f_invert);
  Main->filters.push_back(f_grayscale);
  Main->filters.push_back(f_sepia);
  Main->filters.push_back(f_denoise);
  Main->filters.push_back(f_sharpen);
  Main->filters.push_back(f_edge_detect);
  Main->filters.push_back(f_mosaic);
  Main->filters.push_back(f_resize);
  Main->filters.push_back(f_audio_volume);
  Main->filters.push_back(f_audio_pan);
  Main->filters.push_back(f_audio_eq);
  Main->filters.push_back(f_audio_reverb);
  Main->filters.push_back(f_audio_echo);
}

} // namespace mu::detail
