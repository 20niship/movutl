#pragma once

#include <memory>
#include <movutl/asset/entity.hpp>
#include <movutl/paint/brush.hpp>
#include <string>
#include <vector>

namespace mu {

struct PaintLayer {
  std::string name{"レイヤー"};
  bool visible    = true;
  float opacity   = 1.0f;
  BlendType blend = Blend_Alpha;
  TiledImage img;
};

// Undo/Redoの1単位(1ストローク)。対象レイヤーと、触ったタイルの変更前後
struct PaintHistoryStep {
  int layer = 0;
  std::vector<TilePatch> patches;
};

// 静止画のペイント文書(Compositionとは独立)。レイヤー列 + ストローク単位のUndo/Redo + 保存
// ponytail: レイヤーの追加・削除・並べ替えはUndo対象外(ストロークのみ)。必要になったらPaintHistoryStepを種別付きにする
class PaintDocument {
public:
  PaintDocument(int w, int h) : w_(w), h_(h) {}

  int width() const { return w_; }
  int height() const { return h_; }
  std::vector<PaintLayer> layers;

  int add_layer(const std::string& name);

  // ストローク。begin〜endの間にadd_sample()で点を足す。endでUndo履歴へ積む(Redo履歴は破棄)
  bool begin_stroke(int layer, const BrushSettings& s);
  void add_sample(const PenSample& p);
  void end_stroke();

  bool can_undo() const { return !undo_.empty(); }
  bool can_redo() const { return !redo_.empty(); }
  bool undo();
  bool redo();

  // 可視レイヤーを下から合成してoutへ書く(outはw x hにリサイズされる)。bgは下地色(既定は透明)
  void composite(PixelBuffer8& out, const Vec4b& bg = Vec4b()) const;

  // .mup形式(独自バイナリ: マジック+JSONヘッダ+未圧縮タイル)。失敗時false
  bool save(const std::string& path) const;
  static std::unique_ptr<PaintDocument> load(const std::string& path);

private:
  int w_, h_;
  std::vector<PaintHistoryStep> undo_, redo_;
  std::unique_ptr<BrushStroke> stroke_;
  int stroke_layer_ = -1;
};

} // namespace mu
