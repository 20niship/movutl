#pragma once
#include <algorithm>
#include <cstdint>

namespace mu {

// 簡易線形補間リサンプラ(PCM16 interleaved, rate_ratio=出力レート/入力レート)。ponytail: 帯域制限なし、高音質が必要ならlibswresample等へ差替
inline void audio_resample(const int16_t* in, int in_n, int in_ch, int16_t* out, int out_n, int out_ch, double rate_ratio) {
  if(rate_ratio <= 0.0) rate_ratio = 1.0;
  for(int i = 0; i < out_n; i++) {
    double src_pos = i / rate_ratio;
    int i0         = (int)src_pos;
    double frac    = src_pos - i0;
    int i1         = i0 + 1;
    for(int c = 0; c < out_ch; c++) {
      int sc                      = in_ch <= 0 ? 0 : std::min(c, in_ch - 1);
      int16_t s0                  = (i0 >= 0 && i0 < in_n) ? in[(size_t)i0 * in_ch + sc] : 0;
      int16_t s1                  = (i1 >= 0 && i1 < in_n) ? in[(size_t)i1 * in_ch + sc] : 0;
      out[(size_t)i * out_ch + c] = (int16_t)(s0 + (s1 - s0) * frac);
    }
  }
}

} // namespace mu
