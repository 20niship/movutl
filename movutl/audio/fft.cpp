#include <cmath>
#include <movutl/audio/fft.hpp>
#include <utility>

namespace mu {

void fft_radix2(std::vector<std::complex<float>>& data) {
  size_t n = data.size();
  if(n <= 1) return;

  // bit-reversal permutation
  for(size_t i = 1, j = 0; i < n; i++) {
    size_t bit = n >> 1;
    for(; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if(i < j) std::swap(data[i], data[j]);
  }

  for(size_t len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * (float)M_PI / (float)len;
    std::complex<float> wlen(std::cos(ang), std::sin(ang));
    for(size_t i = 0; i < n; i += len) {
      std::complex<float> w(1.0f, 0.0f);
      for(size_t k = 0; k < len / 2; k++) {
        auto u                = data[i + k];
        auto v                = data[i + k + len / 2] * w;
        data[i + k]           = u + v;
        data[i + k + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
}

void apply_hann_window(std::vector<float>& samples) {
  size_t n = samples.size();
  if(n <= 1) return;
  for(size_t i = 0; i < n; i++) {
    float w = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * (float)i / (float)(n - 1)));
    samples[i] *= w;
  }
}

std::vector<float> compute_spectrum(const int16_t* mono_pcm, int n) {
  std::vector<float> samples(n);
  for(int i = 0; i < n; i++) samples[i] = mono_pcm[i] / 32768.0f;
  apply_hann_window(samples);

  std::vector<std::complex<float>> data(n);
  for(int i = 0; i < n; i++) data[i] = std::complex<float>(samples[i], 0.0f);
  fft_radix2(data);

  std::vector<float> spectrum(n / 2);
  for(int i = 0; i < n / 2; i++) spectrum[i] = std::abs(data[i]) / (float)n * 2.0f;
  return spectrum;
}

} // namespace mu
