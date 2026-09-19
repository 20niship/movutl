#pragma once
#include <imgui.h>
#include <movutl/graphics/GLTexture.hpp>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/transform_gizmo.hpp>

namespace mu {

// Viewer上のマウス位置(コンポ左上原点px)。ステータスバーの座標表示用。Viewer外/コンポ外ではvalid=false
struct ViewerCursor {
  bool valid = false;
  double x   = 0;
  double y   = 0;
};
ViewerCursor& viewer_cursor();

class ViewerWindow final : public UIPanel {
  GLTexture tex;
  float zoom         = 1.0f; // ダブルクリックで1.0にリセット
  ImVec2 pan         = ImVec2(0, 0);
  bool show_checker_ = false; // 透明部分を市松模様で表示(背景色のalphaが0のとき有効)
  bool show_grid_    = false;
  bool show_safe_    = false; // セーフマージン(アクション90%/タイトル80%)
  bool show_center_  = false;
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
