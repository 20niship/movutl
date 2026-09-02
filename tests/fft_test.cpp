#include <cmath>
#include <doctest/doctest.h>
#include <movutl/audio/fft.hpp>
#include <vector>

using namespace mu;

TEST_CASE("compute_spectrum: 440Hzの正弦波のピークが理論値付近のbinに現れる") {
  constexpr int n           = 2048;
  constexpr int sample_rate = 44100;
  constexpr float freq      = 440.0f;

  std::vector<int16_t> pcm(n);
  for(int i = 0; i < n; i++) pcm[i] = (int16_t)(16000.0f * std::sin(2.0f * (float)M_PI * freq * i / sample_rate));

  auto spectrum = compute_spectrum(pcm.data(), n);
  REQUIRE((int)spectrum.size() == n / 2);

  int peak_bin = 0;
  float peak   = 0.0f;
  for(int i = 0; i < (int)spectrum.size(); i++) {
    if(spectrum[i] > peak) {
      peak     = spectrum[i];
      peak_bin = i;
    }
  }

  int expected_bin = (int)std::round(freq * n / sample_rate);
  CHECK(std::abs(peak_bin - expected_bin) <= 1);
}

TEST_CASE("fft_radix2: 直流入力ではDC(bin 0)にのみエネルギーが集中する") {
  constexpr int n = 8;
  std::vector<std::complex<float>> data(n, std::complex<float>(1.0f, 0.0f));
  fft_radix2(data);

  CHECK(std::abs(data[0]) == doctest::Approx((float)n).epsilon(0.01));
  for(int i = 1; i < n; i++) CHECK(std::abs(data[i]) < 1e-3f);
}
