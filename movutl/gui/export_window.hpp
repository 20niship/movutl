#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

// plugin_indexで指定したAppMain::Get()->output_plugins[plugin_index]を選択済みの状態でウィンドウを開く
void open_export_window(int plugin_index);

class ExportWindow final : public UIPanel {
public:
  virtual void Update() override;
  virtual bool always_enabled_during_export() const override;
};

} // namespace mu
