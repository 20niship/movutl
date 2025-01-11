#pragma once

namespace mu {

class UIPanel {
  virtual void Update() {}
};

void init_gui();
void update_gui();

} // namespace mu
