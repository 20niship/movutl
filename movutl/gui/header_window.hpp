#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {
class HeaderWindow final : public UIPanel {
public:
  void header();
  void feader();
  virtual void Update() override;
};
} // namespace mu
