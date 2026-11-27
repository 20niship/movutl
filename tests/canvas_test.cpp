#include "virtual_pen.hpp"
#include <cstring>
#include <doctest/doctest.h>
#include <movutl/core/command.hpp>

using namespace mu;
using mu::test::VirtualPen;

namespace {
bool same_as_full(const PaintDocument& d, const PixelBuffer8& shown, const Vec4b& bg) {
  PixelBuffer8 full;
  d.composite(full, bg);
  return full.size_in_bytes() == shown.size_in_bytes() && std::memcmp(full.data(), shown.data(), full.size_in_bytes()) == 0;
}
} // namespace

TEST_CASE("composite_dirty: 差分合成は全面合成と一致し、変わったタイルだけを返す") {
  const Vec4b bg(255, 255, 255, 255);
  PaintDocument d(300, 200); // タイル境界で割り切れない大きさ
  d.add_layer("a");
  d.add_layer("b");
  PixelBuffer8 shown;
  auto first = d.composite_dirty(shown, bg);
  REQUIRE(first.size() == 1);
  CHECK(first[0].w == 300);
  CHECK(d.composite_dirty(shown, bg).empty());

  BrushSettings s;
  s.size = 12;
  REQUIRE(d.begin_stroke(1, s));
  d.add_sample({20, 20});
  d.add_sample({40, 30});
  auto rects = d.composite_dirty(shown, bg);
  REQUIRE(rects.size() == 1); // 左上の1タイルだけ
  CHECK(rects[0].w == 64);
  CHECK(same_as_full(d, shown, bg));
  d.add_sample({290, 195}); // 右下の端のタイルは文書の大きさで切る
  for(const auto& r : d.composite_dirty(shown, bg)) {
    CHECK(r.x + r.w <= 300);
    CHECK(r.y + r.h <= 200);
  }
  CHECK(same_as_full(d, shown, bg));
  auto edit = d.end_stroke();
  d.composite_dirty(shown, bg);
  REQUIRE(edit);

  edit->undo();
  d.composite_dirty(shown, bg);
  CHECK(same_as_full(d, shown, bg));

  REQUIRE(d.begin_stroke(0, s));
  d.add_sample({100, 100});
  d.composite_dirty(shown, bg);
  d.cancel_stroke();
  CHECK_FALSE(d.composite_dirty(shown, bg).empty());
  CHECK(same_as_full(d, shown, bg));

  // レイヤーの属性変更は全面
  edit->redo();
  auto props    = static_cast<const LayerProps&>(d.layers[1]);
  props.visible = false;
  auto hide     = d.set_layer_props(1, props);
  rects         = d.composite_dirty(shown, bg);
  REQUIRE(rects.size() == 1);
  CHECK(rects[0].w == 300);
  CHECK(same_as_full(d, shown, bg));
}
