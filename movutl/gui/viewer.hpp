#pragma once
#include <movutl/gui/gui.hpp>
#include <movutl/graphics/GLTexture.hpp>

namespace mu {
class ViewerWindow final : public UIPanel {
  GLTexture tex;
public:
  void header();
  virtual void Update() override;
};
} // namespace mu
