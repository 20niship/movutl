#pragma once

#include <imgui.h>
#include <movutl/paint/paint_document.hpp>

namespace mu {

// ponytail: パネルと同じくプロセス終了まで生きるので解放しない。差し替えるようになったらglDeleteTexturesする
class CanvasTexture {
public:
  // 文書に変化があれば更新する。文書がGPUの最大テクスチャサイズを超える場合はfalse
  bool update(PaintDocument& d);
  // 表示倍率に合わせて補間を切り替える(縮小はミップマップ、2倍以上はピクセルをそのまま見せる)。描画の前に呼ぶ
  void set_sampling(float zoom);
  ImTextureID id() const { return (ImTextureID)tex_; }

private:
  unsigned int tex_ = 0;
  PixelBuffer8 buf_; // 合成結果(用紙色の上)
  uint64_t revision_ = 0;
  bool mip_stale_    = true;
};

} // namespace mu
