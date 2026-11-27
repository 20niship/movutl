#pragma once

#include <movutl/gui/gui.hpp>

namespace mu {

// 中央のキャンバス。描画、表示の移動/回転/拡大縮小/反転、ブラシカーソル
Ref<UIPanel> make_canvas_panel();

} // namespace mu
