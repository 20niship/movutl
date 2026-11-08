#pragma once

#include <algorithm>
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>
#include <type_traits>

// 8bit RGBA用の合成カーネル。Image(動画/画像合成)とペイント層(raster/paint)で共有する

namespace mu {

inline int div255(int x) { return (x + 128 + ((x + 128) >> 8)) >> 8; } // x/255の四捨五入(x<=65535)、除算なし

// BlendTypeに応じたチャンネル合成式(0-255域)。dはブレンド前の合成先(base)、sはブレンド元(blend)の値
// BlendTypeをテンプレート引数にして、ピクセルごとのswitchを内側ループから外す(コンパイル時に式が確定する)
template <BlendType B> inline int blend_channel(int d, int s) {
  if constexpr(B == Blend_Add)
    return std::min(255, d + s);
  else if constexpr(B == Blend_Sub)
    return std::max(0, d - s);
  else if constexpr(B == Blend_Mul)
    return div255(d * s);
  else if constexpr(B == Blend_Div)
    return s > 0 ? std::min(255, d * 255 / s) : 255;
  else if constexpr(B == Blend_Screen)
    return d + s - d * s / 255; // 255-(255-d)(255-s)/255(整数除算)と同値で乗算1回
  else if constexpr(B == Blend_Overlay)
    return d < 128 ? 2 * d * s / 255 : 255 - 2 * (255 - d) * (255 - s) / 255;
  else if constexpr(B == Blend_Darken)
    return std::min(d, s);
  else if constexpr(B == Blend_Lighten)
    return std::max(d, s);
  else if constexpr(B == Blend_HardLight)
    return s < 128 ? 2 * d * s / 255 : 255 - 2 * (255 - d) * (255 - s) / 255;
  else
    return s;
}

/// srcをdstへ合成する。am256=alpha_mul*256(固定小数)。整数演算のみ
template <BlendType B> inline void blend_pixel_t(Vec4b& d, const Vec4b& src, int am256) {
  int a = ((src[3] + (src[3] >> 7)) * am256) >> 8; // 0..256(src[3]=255を256へ写像し、不透明×alpha_mul=1が厳密に256になる)
  if(a <= 0) return;
  if constexpr(B == Blend_Alpha) {
    if(a >= 256) {
      d = src;
      return;
    }
  }
  for(int c = 0; c < 3; c++) {
    int dc = d[c];
    int bl = blend_channel<B>(dc, src[c]);
    d[c]   = (unsigned char)(dc + (((bl - dc) * a) >> 8));
  }
  d[3] = (unsigned char)(d[3] + (((255 - d[3]) * a) >> 8));
}

inline int alpha_to_256(float alpha_mul) { return (int)(std::clamp(alpha_mul, 0.0f, 1.0f) * 256.0f + 0.5f); }

/// blendの種類を1回だけ分岐してfn(integral_constant<BlendType>)へ渡す
template <typename F> inline void with_blend(BlendType blend, F&& fn) {
  switch(blend) {
    case Blend_Add: fn(std::integral_constant<BlendType, Blend_Add>{}); break;
    case Blend_Sub: fn(std::integral_constant<BlendType, Blend_Sub>{}); break;
    case Blend_Mul: fn(std::integral_constant<BlendType, Blend_Mul>{}); break;
    case Blend_Div: fn(std::integral_constant<BlendType, Blend_Div>{}); break;
    case Blend_Screen: fn(std::integral_constant<BlendType, Blend_Screen>{}); break;
    case Blend_Overlay: fn(std::integral_constant<BlendType, Blend_Overlay>{}); break;
    case Blend_Darken: fn(std::integral_constant<BlendType, Blend_Darken>{}); break;
    case Blend_Lighten: fn(std::integral_constant<BlendType, Blend_Lighten>{}); break;
    case Blend_HardLight: fn(std::integral_constant<BlendType, Blend_HardLight>{}); break;
    default: fn(std::integral_constant<BlendType, Blend_Alpha>{}); break;
  }
}

// ストレートアルファ同士の通常合成(Porter-Duff over)。sa=srcの実効不透明度(0〜1、src[3]とは別にレイヤー不透明度等を掛けた値)
// ponytail: 浮動小数。ペイント層は半透明の重ね塗りが多く、blend_pixel_tはdstが透明なピクセルで色が黒へ引かれるため別に持つ
inline void blend_over_straight(Vec4b& d, const Vec4b& s, float sa) {
  const float da = d[3] / 255.0f;
  const float oa = sa + da * (1.0f - sa);
  if(oa <= 0.0f) {
    d = Vec4b();
    return;
  }
  for(int c = 0; c < 3; c++) d[c] = (unsigned char)std::clamp((s[c] * sa + d[c] * da * (1.0f - sa)) / oa + 0.5f, 0.0f, 255.0f);
  d[3] = (unsigned char)std::clamp(oa * 255.0f + 0.5f, 0.0f, 255.0f);
}

} // namespace mu
