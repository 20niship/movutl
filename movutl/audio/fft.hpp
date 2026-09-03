#pragma once
#include <complex>
#include <cstdint>
#include <vector>

namespace mu {

// in-place radix-2 Cooley-Tukey FFT。data.size()は2の冪であること(呼び出し側が保証)
void fft_radix2(std::vector<std::complex<float>>& data);

// Hann窓をin-placeで適用する
void apply_hann_window(std::vector<float>& samples);

// mono_pcm(サイズn、nは2の冪)から振幅スペクトル(n/2点、0-1正規化)を計算する
std::vector<float> compute_spectrum(const int16_t* mono_pcm, int n);

} // namespace mu
