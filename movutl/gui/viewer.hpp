#pragma once
#include <movutl/graphics/GLTexture.hpp>
#include <movutl/gui/gui.hpp>

namespace mu {
class ViewerWindow final : public UIPanel {
  GLTexture tex;

public:
  void header();
  virtual void Update() override;
};
} // namespace mu
