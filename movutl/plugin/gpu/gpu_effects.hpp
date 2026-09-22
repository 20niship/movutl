#pragma once
/**
 * CPU版フィルタ(movutl/plugin/default/image_tone_filter.cpp, image_color_filter.cpp)と同じ計算結果になる
 * GPU(compute shader)版。FilterPluginTableへの自動組み込みは行わない(既存フィルタ呼び出し経路は変更しない。
 * ponytail: GPU/CPU自動切替の仕組みはこの関数群を使う側が要る時に足す。今は直接呼べれば十分)
 */
#include <movutl/asset/image.hpp>

namespace mu::detail {

// f_invert(image_tone_filter.cpp)と同じ: RGBを反転。invert_alpha=trueならアルファも反転
bool gpu_invert(Image& img, bool invert_alpha = false);

// f_color_correction(image_color_filter.cpp)と同じ引数の意味。hue/saturation使用時はOpenCV HSV_FULL内部実装とのビット一致は保証しない(セクタ境界が256/6で割り切れずずれるため)
bool gpu_color_correction(Image& img, float brightness = 100.0f, float contrast = 100.0f, float hue = 0.0f, float saturation = 100.0f);

// 新規エフェクト「並べて配置」。元画像をnx x ny個並べて同じ出力サイズに敷き詰める(CPU版はimage_tile_filter.cpp)
bool gpu_tile(Image& img, int nx, int ny);

} // namespace mu::detail
