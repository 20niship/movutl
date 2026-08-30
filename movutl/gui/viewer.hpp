#pragma once
#include <imgui.h>
#include <movutl/graphics/GLTexture.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {
class ViewerWindow final : public UIPanel {
  GLTexture tex;
  float zoom = 1.0f; // ダブルクリックで1.0にリセット
  ImVec2 pan = ImVec2(0, 0);
  cutil::WeakPtr<Image> last_bound_frame_; // texに束縛中のFrameCache由来Imageの識別用

public:
  void header();
  virtual void Update() override;
};
} // namespace mu
