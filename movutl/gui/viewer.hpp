#pragma once
#include <imgui.h>
#include <movutl/graphics/GLTexture.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/transform_gizmo.hpp>

namespace mu {
class ViewerWindow final : public UIPanel {
  GLTexture tex;
  float zoom = 1.0f; // ダブルクリックで1.0にリセット
  ImVec2 pan = ImVec2(0, 0);
  cutil::WeakPtr<Image> last_bound_frame_; // texに束縛中のFrameCache由来Imageの識別用

  // ギズモのドラッグ中状態。s0/m0はドラッグ開始時の変換とマウス位置(コンポ座標)で、毎フレームこれらと現在のマウス位置から結果を求める
  struct GizmoDrag {
    GizmoPart part = GizmoPart::None;
    int corner     = -1;
    GizmoPt m0;
    GizmoXform s0;
    Ref<Entity> entt;
  } drag_;

public:
  void header();
  virtual void Update() override;
};
} // namespace mu
