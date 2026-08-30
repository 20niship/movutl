#pragma once
#include <movutl/gui/gui.hpp>

namespace mu {

// バックグラウンドレンダーワーカーの稼働状況を表示する開発者向けウィンドウ
class DeveloperWindow final : public UIPanel {
public:
  virtual void Update() override;
};

} // namespace mu
