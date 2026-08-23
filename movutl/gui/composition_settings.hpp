#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

// Project::compos_のうちflag & Composition::setting_dialogが立っているものの設定ウィンドウを表示する
class CompositionSettingsWindow final : public UIPanel {
public:
  virtual void Update() override;
};

} // namespace mu
