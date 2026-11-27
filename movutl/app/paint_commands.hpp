#pragma once

#include <memory>
#include <movutl/core/command.hpp>
#include <movutl/paint/paint_edit.hpp>

namespace mu {

// PaintEditを動画編集と共通のundo履歴に載せる
class PaintEditCommand final : public mCommand {
public:
  explicit PaintEditCommand(std::unique_ptr<PaintEdit> edit) : edit_(std::move(edit)) { name = edit_->name(); }
  void on_undo() override { edit_->undo(); }
  void on_redo() override { edit_->redo(); }
  bool undoable() const override { return true; }
  bool expired() const override { return edit_->expired(); }
  bool modifies_project() const override { return false; } // 保存状態はPaintEditorStateが持つ

private:
  std::unique_ptr<PaintEdit> edit_;
};

// 適用済みのeditを履歴へ積む。nullptrは何もしないので、PaintDocumentの戻り値をそのまま渡せる
void push_paint_edit(std::unique_ptr<PaintEdit> edit);

namespace detail {
void register_paint_commands(); // ツール切替・ブラシサイズ/不透明度・描画色のコマンド
}

} // namespace mu
