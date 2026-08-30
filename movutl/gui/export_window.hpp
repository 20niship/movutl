#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

void open_export_window();

class ExportWindow final : public UIPanel {
public:
  virtual void Update() override;
};

} // namespace mu
