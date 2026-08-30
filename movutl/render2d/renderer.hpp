#pragma once

#include <movutl/asset/composition.hpp>

namespace mu {

// 将来のGPU(Vulkan)レンダラー追加(Issue#3 6.2)に備えた共通インタフェース
class Renderer {
public:
  virtual ~Renderer() = default;

  // comp->mtxを内部でlockするため呼び出し側でのlockは不要
  virtual bool render_frame(Composition* comp, int frame, Ref<Image>& out) = 0;
};

// 既存のopencv/CPUによる画像処理でレンダリングする実装
class CPURenderer : public Renderer {
public:
  bool render_frame(Composition* comp, int frame, Ref<Image>& out) override;
};

} // namespace mu
