#include <movutl/app/app_impl.hpp>
#include <movutl/plugin/default/audio_filters.hpp>
#include <movutl/plugin/default/audio_volume_filter.hpp>
#include <movutl/plugin/default/image_advcolor_filter.hpp>
#include <movutl/plugin/default/image_blur_filter.hpp>
#include <movutl/plugin/default/image_color_filter.hpp>
#include <movutl/plugin/default/image_distortion_filter.hpp>
#include <movutl/plugin/default/image_effect_filter.hpp>
#include <movutl/plugin/default/image_glow_filter.hpp>
#include <movutl/plugin/default/image_gradient_filter.hpp>
#include <movutl/plugin/default/image_key_filter.hpp>
#include <movutl/plugin/default/image_outline_filter.hpp>
#include <movutl/plugin/default/image_posterize_filter.hpp>
#include <movutl/plugin/default/image_tile_filter.hpp>
#include <movutl/plugin/default/image_tone_filter.hpp>
#include <movutl/plugin/default/image_vintage_filter.hpp>
#include <movutl/plugin/gpu/gpu_effects.hpp>
#include <movutl/plugin/plugin.hpp>

namespace mu::detail {

namespace {
// fn_proc_gpu配線用の薄いラッパ(gpu_effects.hpp)。対応する3フィルタのみ(過剰実装しない)
bool fn_proc_invert_gpu(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  return gpu_invert(*fpip->img, cutil::get_or<bool>(p, "invert_alpha", false));
}
bool fn_proc_color_correction_gpu(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  return gpu_color_correction(*fpip->img, cutil::get_or<float>(p, "brightness", 100.0f), cutil::get_or<float>(p, "contrast", 100.0f), cutil::get_or<float>(p, "hue", 0.0f), cutil::get_or<float>(p, "saturation", 100.0f));
}
bool fn_proc_tile_gpu(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  return gpu_tile(*fpip->img, cutil::get_or<int32_t>(p, "nx", 2), cutil::get_or<int32_t>(p, "ny", 2));
}
} // namespace

void register_default_plugins() {
  // video_reader は plugins/video_reader/*.mso として外部プラグイン経由で登録される
}

void register_default_filters() {
  auto Main                      = AppMain::Get();
  f_color_correction.fn_proc_gpu = fn_proc_color_correction_gpu;
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
  f_invert.fn_proc_gpu = fn_proc_invert_gpu;
  Main->filters.push_back(f_invert);
  Main->filters.push_back(f_grayscale);
  Main->filters.push_back(f_sepia);
  f_tile.fn_proc_gpu = fn_proc_tile_gpu;
  Main->filters.push_back(f_tile);
  Main->filters.push_back(f_denoise);
  Main->filters.push_back(f_sharpen);
  Main->filters.push_back(f_edge_detect);
  Main->filters.push_back(f_mosaic);
  Main->filters.push_back(f_resize);
  Main->filters.push_back(f_four_color_gradient);
  Main->filters.push_back(f_radial_gradient);
  Main->filters.push_back(f_diagonal_clip);
  Main->filters.push_back(f_circle_clip);
  Main->filters.push_back(f_vignette);
  Main->filters.push_back(f_film_grain);
  Main->filters.push_back(f_scanline);
  Main->filters.push_back(f_vhs_noise);
  Main->filters.push_back(f_posterize);
  Main->filters.push_back(f_binarize);
  Main->filters.push_back(f_emboss);
  Main->filters.push_back(f_halftone);
  Main->filters.push_back(f_lens_distortion);
  Main->filters.push_back(f_ripple);
  Main->filters.push_back(f_wave_distortion);
  Main->filters.push_back(f_kaleidoscope);
  Main->filters.push_back(f_color_balance);
  Main->filters.push_back(f_color_lut);
  Main->filters.push_back(f_soft_focus);
  Main->filters.push_back(f_interlace_shift);
  Main->filters.push_back(f_audio_volume);
  Main->filters.push_back(f_audio_pan);
  Main->filters.push_back(f_audio_eq);
  Main->filters.push_back(f_audio_reverb);
  Main->filters.push_back(f_audio_echo);
}

} // namespace mu::detail
