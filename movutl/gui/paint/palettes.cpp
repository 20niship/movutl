#include <filesystem>
#include <movutl/core/logger.hpp>
#include <movutl/gui/paint/palettes.hpp>
#include <movutl/gui/workspace_presets.hpp>

namespace mu {

void BrushStore::load(BrushLibrary& lib) {
  std::error_code ec;
  if(std::filesystem::exists(path, ec) && !lib.load(path)) {
    LOG_F(WARNING, "ブラシ設定を読み込めません。%s.bak へ退避します", path.c_str());
    std::filesystem::rename(path, path + ".bak", ec);
  }
  saved = lib.dump();
}

bool BrushStore::flush(const BrushLibrary& lib) {
  auto now = lib.dump();
  if(now == saved || !lib.save(path)) return false;
  saved = std::move(now);
  return true;
}

namespace {

// movutl_cnf.jsonと同じく作業ディレクトリに置く
constexpr const char* kBrushFile = "movutl_brushes.json";

// window_nameが空なので、どのワークスペースでも毎フレーム呼ばれる
struct BrushAutoSave final : UIPanel {
  BrushStore store{kBrushFile, {}};
  double next = 0;
  BrushAutoSave() { store.load(paint_editor().brushes); }
  bool always_enabled_during_export() const override { return true; }
  void Update() override {
    // 毎フレームの比較は無駄なので1秒おき。終了するフレームでは必ず書く
    if(!GUIManager::Get()->should_close && ImGui::GetTime() < next) return;
    next = ImGui::GetTime() + 1.0;
    store.flush(paint_editor().brushes);
  }
};

} // namespace

void register_paint_palettes() {
  auto& panels = GUIManager::Get()->panels;
  // 先頭に置く(update_gui_panels()が指定するWindowClassを後続のパネルが上書きできるように)
  panels.insert(panels.begin(), cutil::make_ref<BrushAutoSave>());
  set_paint_panel("paint_tool", cutil::make_ref<ToolPanel>());
  set_paint_panel("subtool", cutil::make_ref<SubToolPanel>());
  set_paint_panel("tool_prop", cutil::make_ref<ToolPropertyPanel>());
  set_paint_panel("brush_size", cutil::make_ref<BrushSizePanel>());
  set_paint_panel("color", cutil::make_ref<ColorPanel>());
  set_paint_panel("layers", cutil::make_ref<LayerPanel>());
  set_paint_panel("layer_prop", cutil::make_ref<LayerPropertyPanel>());
  set_paint_panel("navigator", cutil::make_ref<NavigatorPanel>());
  set_paint_panel("history", cutil::make_ref<HistoryPanel>());
  set_paint_panel("command_bar", cutil::make_ref<CommandBarPanel>());
}

} // namespace mu
