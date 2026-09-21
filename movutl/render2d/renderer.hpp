#pragma once

#include <movutl/asset/composition.hpp>

namespace mu {

// CPU/GPU(Vulkan)などレンダラー実装の共通インタフェース(Issue#3 6.2)。実装はrenderer_registry.hppで名前登録して切り替える
class Renderer {
public:
  virtual ~Renderer() = default;

  // comp->mtxを内部でlockするため呼び出し側でのlockは不要
  // transparent_bg=trueはCompoRefEntt等のネスト参照時用で、comp->bg_colorを無視し完全透明で背景を敷く(直接表示/書き出し時の不透明背景と区別)
  virtual bool render_frame(Composition* comp, int frame, Ref<Image>& out, bool transparent_bg) = 0;
  bool render_frame(Composition* comp, int frame, Ref<Image>& out) { return render_frame(comp, frame, out, false); }
};

// 既存のopencv/CPUによる画像処理でレンダリングする実装
class CPURenderer : public Renderer {
public:
  using Renderer::render_frame;
  bool render_frame(Composition* comp, int frame, Ref<Image>& out, bool transparent_bg) override;
};

} // namespace mu
