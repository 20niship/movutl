#pragma once

#include <movutl/paint/stabilizer.hpp>
#include <movutl/paint/tools/tool.hpp>

namespace mu {

// ブラシで描くツール(ペン・鉛筆・筆・エアブラシ・消しゴム)。描き方の違いはBrushPresetで表す
class BrushTool : public Tool {
public:
  using Tool::Tool;
  bool uses_brush() const override { return true; }
  bool press(const ToolContext& c, const PenSample& p) override {
    if(!c.doc->begin_stroke(c.layer, c.brush)) return false;
    stabilizer_ = Stabilizer(c.brush.stabilization);
    c.doc->add_sample(stabilizer_.add(p));
    return true;
  }
  void drag(const ToolContext& c, const PenSample& p) override { c.doc->add_sample(stabilizer_.add(p)); }
  void tick(const ToolContext& c, float dt) override { c.doc->tick(dt); }
  std::unique_ptr<PaintEdit> release(const ToolContext& c) override {
    for(const auto& p : stabilizer_.finish()) c.doc->add_sample(p);
    return c.doc->end_stroke();
  }
  void cancel(const ToolContext& c) override { c.doc->cancel_stroke(); }

private:
  Stabilizer stabilizer_;
};

} // namespace mu
