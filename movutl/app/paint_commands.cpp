#include <movutl/app/paint_commands.hpp>
#include <movutl/gui/paint/editor_state.hpp>

namespace mu {

void push_paint_edit(std::unique_ptr<PaintEdit> edit) {
  if(edit) push_undo_command(cutil::make_ref<PaintEditCommand>(std::move(edit)));
}

namespace detail {

namespace {

// 共有のエディタ状態を操作するだけのコマンド(undo対象外)
struct EditorCommand final : mCommand {
  explicit EditorCommand(bool (*fn)(PaintEditorState&)) : fn_(fn) {}
  CommandStatus on_start() override { return fn_(paint_editor()) ? CommandStatus::Finished : CommandStatus::Failed; }
  bool (*fn_)(PaintEditorState&);
};

void add(const char* id, const char* name, const char* desc, const char* key, const char* context, bool (*fn)(PaintEditorState&)) { register_command<EditorCommand>({id, name, desc, key, {}, context}, fn); }

} // namespace

void register_paint_commands() {
  add("paint_tool_pen", "ペン・鉛筆", "ペン/鉛筆ツールに切り替える(繰り返すと順送り)", "p", "", [](PaintEditorState& s) { return s.tools.select_by_key("p"); });
  add("paint_tool_brush", "筆・エアブラシ", "筆/エアブラシ/デコレーションに切り替える(繰り返すと順送り)", "b", "", [](PaintEditorState& s) { return s.tools.select_by_key("b"); });
  add("paint_tool_eraser", "消しゴム", "消しゴムツールに切り替える", "e", "", [](PaintEditorState& s) { return s.tools.select_by_key("e"); });
  add("paint_tool_eyedropper", "スポイト", "スポイトツールに切り替える", "i", "", [](PaintEditorState& s) { return s.tools.select_by_key("i"); });

  add("paint_brush_size_down", "ブラシサイズを小さく", "ブラシサイズを1段階小さくする", "[", "canvas", [](PaintEditorState& s) { return s.step_brush_size(-1); });
  add("paint_brush_size_up", "ブラシサイズを大きく", "ブラシサイズを1段階大きくする", "]", "canvas", [](PaintEditorState& s) { return s.step_brush_size(+1); });
  add("paint_opacity_down", "不透明度を下げる", "ブラシの不透明度を10%下げる", "ctrl+[", "canvas", [](PaintEditorState& s) { return s.step_brush_opacity(-1); });
  add("paint_opacity_up", "不透明度を上げる", "ブラシの不透明度を10%上げる", "ctrl+]", "canvas", [](PaintEditorState& s) { return s.step_brush_opacity(+1); });

  add("paint_swap_color", "メイン/サブカラー切替", "描画色をメインカラーとサブカラーで切り替える", "x", "canvas", [](PaintEditorState& s) {
    s.swap_color();
    return true;
  });
  add("paint_transparent_color", "透明色切替", "描画色の透明色を切り替える", "c", "canvas", [](PaintEditorState& s) {
    s.toggle_transparent();
    return true;
  });
}

} // namespace detail
} // namespace mu
