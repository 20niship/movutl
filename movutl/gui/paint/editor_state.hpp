#pragma once

#include <memory>
#include <movutl/gui/paint/canvas_view.hpp>
#include <movutl/input/pen_input.hpp>
#include <movutl/paint/brush_library.hpp>
#include <movutl/paint/paint_document.hpp>
#include <movutl/paint/tools/tool_manager.hpp>
#include <string>
#include <vector>

namespace mu {

enum class PaintColorSlot { Main, Sub, Transparent };

// ペイント画面の状態(キャンバスと各パレットが共有する)。ImGuiに依存しない。GUIスレッドから使う
class PaintEditorState {
public:
  static constexpr int kMaxCanvasSize     = 16384;
  static constexpr float kMaxBrushSize    = 2000.0f;
  static constexpr float kMinBrushOpacity = 0.01f;

  ToolManager tools;
  BrushLibrary brushes = BrushLibrary::standard();

  Vec4b main_color{0, 0, 0, 255};
  Vec4b sub_color{255, 255, 255, 255};
  PaintColorSlot color_slot = PaintColorSlot::Main;
  std::vector<Vec4b> color_history; // 描画に使った色(新しい順、重複なし)
  static constexpr size_t kMaxColorHistory = 32;

  CanvasView view; // キャンバスの表示位置。view_pos/view_size/canvas_sizeはキャンバスパネルが毎フレーム設定する

  // ---- 文書 ----
  PaintDocument* doc() const { return doc_.get(); } // 未作成ならnullptr
  bool new_document(int w, int h);                  // レイヤー1枚の文書を作る。大きさが不正ならfalse
  bool open(const std::string& path);               // 失敗時は今の文書のまま
  bool save();                                      // 保存先が未定(pathが空)ならfalse
  bool save_as(const std::string& path);
  const std::string& path() const { return path_; }
  bool modified() const { return doc_ && doc_->revision() != saved_revision_; } // 破棄する前の確認はGUIで行う

  // ---- レイヤー(undoできる) ----
  int active_layer() const; // 常に有効な範囲。文書が無ければ-1
  void set_active_layer(int i) { layer_ = i; }
  bool add_layer(); // アクティブレイヤーの上に追加して選ぶ
  bool remove_layer();
  bool move_layer(int to); // アクティブレイヤーをtoへ移す
  bool set_layer_props(int i, const LayerProps& p);

  // ---- ブラシと色 ----
  BrushPreset* preset();          // 使用中ツールの選択サブツール。ブラシを使わないツールならnullptr
  BrushSettings brush_settings(); // preset()の現在値に描画色を反映したもの(透明色なら消しゴムになる)
  bool step_brush_size(int dir);  // dir>0で大きく。ブラシを使わないツールならfalse
  bool step_brush_opacity(int dir);
  Vec4b color() const { return color_slot == PaintColorSlot::Sub ? sub_color : main_color; } // 透明色の時はメインカラー
  void set_color(const Vec4b& c);                                                            // 選択中の色を変える。透明色の時はメインカラーに入れてメインを選ぶ
  void swap_color();                                                                         // X: メイン⇔サブ(透明色からはメインへ)
  void toggle_transparent();                                                                 // C: 透明色⇔直前の色

  // ---- 入力 ----
  bool pointer(const PenEvent& e); // sampleはキャンバス座標。Downで操作を開始できなければfalse
  void tick(float dt) { tools.tick(dt); }
  void cancel() { finish(false); } // 操作中のストロークを破棄する(Esc)

private:
  void set_document(std::unique_ptr<PaintDocument> d, std::string path);
  bool commit(std::unique_ptr<PaintEdit> e); // 共通の履歴へ積む。nullptrならfalse
  void finish(bool apply);                   // 操作を終える

  std::unique_ptr<PaintDocument> doc_;
  std::string path_;
  uint64_t saved_revision_  = 0;
  int layer_                = 0;
  PaintColorSlot prev_slot_ = PaintColorSlot::Main; // 透明色にする前の色
  bool pen_eraser_          = false;                // ペンの消しゴム側で一時切替中
};

PaintEditorState& paint_editor(); // GUIとコマンドが共有するインスタンス

} // namespace mu
