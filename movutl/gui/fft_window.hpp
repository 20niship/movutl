#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

class FFTWindow final : public UIPanel {
public:
  virtual void Update() override;
};

} // namespace mu
