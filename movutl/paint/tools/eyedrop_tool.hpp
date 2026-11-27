#pragma once

#include <cmath>
#include <movutl/paint/tools/tool.hpp>

namespace mu {

// スポイト: 画面に見えている色(全レイヤー合成)を不透明色として拾う。透明な所では何も拾わない
class EyedropTool : public Tool {
public:
  using Tool::Tool;
  bool press(const ToolContext& c, const PenSample& p) override {
    pick(c, p);
    return true;
  }
  void drag(const ToolContext& c, const PenSample& p) override { pick(c, p); }

private:
  static void pick(const ToolContext& c, const PenSample& p) {
    Vec4b col = c.doc->pick((int)std::floor(p.x), (int)std::floor(p.y));
    if(col[3] == 0 || !c.on_pick_color) return;
    col[3] = 255;
    c.on_pick_color(col);
  }
};

} // namespace mu
