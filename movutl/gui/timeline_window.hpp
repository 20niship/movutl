#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {
class TimelineWindow final : public UIPanel {
public:
  void header();
  void feader();
  virtual void Update() override;
};
} // namespace mu
