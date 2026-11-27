#pragma once

#include <imgui.h>
#include <memory>
#include <movutl/gui/gui.hpp>
#include <movutl/gui/paint/editor_state.hpp>
#include <optional>
#include <string>

namespace mu {

// ペイントの基本パレット。共有状態はpaint_editor()

// 全パレットを登録し、ブラシ設定を読み込む(detail::init_workspaces()より後に呼ぶ)
void register_paint_palettes();

// ブラシ設定の保存先。起動時に読み込み、変更があれば書き戻す
struct BrushStore {
  std::string path;
  std::string saved; // ファイルと一致している内容
  // 読めないファイルは上書きで失わないように<path>.bakへ退避する
  void load(BrushLibrary& lib);
  bool flush(const BrushLibrary& lib); // 変更があれば保存する。保存したらtrue
};

// 次の幅wの部品が右端に収まるなら同じ行に置く(直前の部品の直後に呼ぶ)
inline void flow_same_line(float w) {
  const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
  if(ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + w <= right) ImGui::SameLine();
}

// 8bit色とImGuiの色の変換
inline ImVec4 to_imvec4(const Vec4b& c) { return ImVec4(c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f, c[3] / 255.0f); }

struct ToolPanel final : UIPanel {
  void Update() override;
};
struct SubToolPanel final : UIPanel {
  void Update() override;
  std::string shown_;    // タブを合わせ済みの"ツール/グループ"
  std::string scrolled_; // スクロールを合わせ済みの選択
};
struct ToolPropertyPanel final : UIPanel {
  void Update() override;
};
struct BrushSizePanel final : UIPanel {
  void Update() override;
};
struct ColorPanel final : UIPanel {
  void Update() override;
  float rgb_[3] = {0, 0, 0}; // ピッカー操作中の色。8bitへ丸めると色相がずれるので保持する
};
struct LayerPanel final : UIPanel {
  void Update() override;

private:
  // 不透明度のドラッグと名前変更は、終わるまで対象を覚えておく
  struct Target {
    std::weak_ptr<PaintDocument*> doc;
    int index = 0;
  };
  struct Drag : Target {
    LayerProps before;
  };
  std::optional<Drag> drag_;
  std::optional<Target> rename_;
  char name_[256]    = {};
  bool focus_rename_ = false;
};
struct LayerPropertyPanel final : UIPanel {
  void Update() override;
};
struct NavigatorPanel final : UIPanel {
  void Update() override;

private:
  struct Texture; // GLのテクスチャと縮小画像
  std::shared_ptr<Texture> tex_;
};
struct HistoryPanel final : UIPanel {
  void Update() override;
  int last_applied_ = -1;
  size_t last_size_ = 0;
};
struct CommandBarPanel final : UIPanel {
  void Update() override;
};

} // namespace mu
