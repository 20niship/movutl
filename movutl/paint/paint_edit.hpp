#pragma once

#include <functional>
#include <memory>
#include <movutl/raster/tiled_image.hpp>
#include <string>
#include <vector>

namespace mu {

class PaintDocument;

// ペイント文書への1操作分のundo/redo。操作を適用した後の状態で作られる
// 文書は弱参照なので、文書が先に破棄されてもundo/redoは何もしないだけで安全
class PaintEdit {
public:
  virtual ~PaintEdit() = default;
  virtual void undo()  = 0;
  virtual void redo()  = 0;
  const std::string& name() const { return name_; } // 履歴に出す名前
  bool expired() const { return doc_.expired(); }   // 対象の文書が破棄された

protected:
  PaintEdit(const PaintDocument& d, std::string name);
  // 書き換える直前に呼ぶ。描画中のストロークは中止し、revisionを進める。文書が破棄済みならnullptr
  PaintDocument* target() const;

private:
  std::weak_ptr<PaintDocument*> doc_;
  std::string name_;
};

// 1ストローク分: 触ったタイルの変更前後
class StrokeEdit final : public PaintEdit {
public:
  StrokeEdit(const PaintDocument& d, int layer, std::vector<TilePatch> patches) : PaintEdit(d, "ストローク"), layer_(layer), patches_(std::move(patches)) {}
  void undo() override { apply(false); }
  void redo() override { apply(true); }

private:
  void apply(bool after);
  int layer_;
  std::vector<TilePatch> patches_;
};

// レイヤーの追加/削除/並べ替え/属性変更。どれも「もう一度呼ぶと元に戻る」入れ替え操作1つで表せる
class LayerEdit final : public PaintEdit {
public:
  LayerEdit(const PaintDocument& d, std::string name, std::function<void(PaintDocument&)> toggle) : PaintEdit(d, std::move(name)), toggle_(std::move(toggle)) {}
  void undo() override { run(); }
  void redo() override { run(); }

private:
  void run() {
    if(auto* d = target()) toggle_(*d);
  }
  std::function<void(PaintDocument&)> toggle_;
};

} // namespace mu
