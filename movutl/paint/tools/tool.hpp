#pragma once

#include <functional>
#include <memory>
#include <movutl/paint/paint_document.hpp>
#include <string>

namespace mu {

// ツールの1操作(押してから離すまで)に必要なもの。押した時点の値が操作の最後まで使われる
struct ToolContext {
  PaintDocument* doc = nullptr;
  int layer          = 0;
  BrushSettings brush;                             // 描画色まで反映済み
  std::function<void(const Vec4b&)> on_pick_color; // スポイトが拾った色を受け取る
};

// キャンバス上の操作。座標はキャンバス座標。UIに依存しない
class Tool {
public:
  Tool(std::string id, std::string name, std::string key) : id(std::move(id)), name(std::move(name)), key(std::move(key)) {}
  virtual ~Tool() = default;

  const std::string id;   // "pen"など。BrushPreset::toolと対応する
  const std::string name; // 表示名
  const std::string key;  // 切替キー。同じキーのツールは順送りになる

  virtual bool uses_brush() const { return false; } // サブツール/ブラシサイズ/ブラシカーソルを出すか

  virtual bool press(const ToolContext& c, const PenSample& p) = 0; // falseなら操作を始めない
  virtual void drag(const ToolContext&, const PenSample&) {}
  virtual void tick(const ToolContext&, float) {} // 押している間、毎フレーム(秒)
  virtual std::unique_ptr<PaintEdit> release(const ToolContext&) { return nullptr; }
  virtual void cancel(const ToolContext&) {}
};

} // namespace mu
