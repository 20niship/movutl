#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

// 選択中のMidiEntt(EntityType_Midi)を編集するピアノロールウィンドウ
class PianoRollWindow final : public UIPanel {
public:
  virtual void Update() override;
};

} // namespace mu
