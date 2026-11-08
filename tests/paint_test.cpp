#include <doctest/doctest.h>
#include <filesystem>
#include <movutl/paint/paint_document.hpp>

using namespace mu;

namespace {
BrushSettings black(float size) {
  BrushSettings s;
  s.size     = size;
  s.hardness = 1.0f;
  s.color    = Vec4b{0, 0, 0, 255};
  return s;
}
void hline(PaintDocument& d, int layer, const BrushSettings& s, float y, float p = 1.0f) {
  REQUIRE(d.begin_stroke(layer, s));
  for(float x = 10; x <= 90; x += 5) d.add_sample({x, y, p});
  d.end_stroke();
}
} // namespace

TEST_CASE("TiledImage: 描いたタイルだけ確保され、未描画は透明") {
  TiledImage t(200, 200);
  CHECK(t.tile_count() == 0);
  CHECK(t.get(5, 5)[3] == 0);
  t.at(70, 5) = Vec4b{1, 2, 3, 255};
  CHECK(t.tile_count() == 1);
  CHECK(t.get(70, 5)[2] == 3);
  CHECK(t.get(0, 0)[3] == 0);
  CHECK(t.get(-1, 0)[3] == 0); // 範囲外は透明
}

TEST_CASE("BrushStroke: 太さと筆圧") {
  PaintDocument d(100, 100);
  int l = d.add_layer("a");
  hline(d, l, black(10), 20);
  CHECK(d.layers[l].img.get(50, 20)[3] == 255);
  CHECK(d.layers[l].img.get(50, 26)[3] == 0); // 半径5の外

  int l2 = d.add_layer("b");
  hline(d, l2, black(10), 50, 0.3f);            // 筆圧0.3 → 直径3
  CHECK(d.layers[l2].img.get(50, 50)[3] > 200); // 細い線は縁のAAでほぼ不透明
  CHECK(d.layers[l2].img.get(50, 53)[3] == 0);
}

TEST_CASE("PaintDocument: ストロークのundo/redo") {
  PaintDocument d(100, 100);
  int l = d.add_layer("a");
  hline(d, l, black(8), 30);
  CHECK(d.layers[l].img.get(50, 30)[3] == 255);
  CHECK(d.undo());
  CHECK(d.layers[l].img.get(50, 30)[3] == 0);
  CHECK(d.layers[l].img.tile_count() == 0);
  CHECK_FALSE(d.undo());
  CHECK(d.redo());
  CHECK(d.layers[l].img.get(50, 30)[3] == 255);
  hline(d, l, black(8), 60);
  CHECK_FALSE(d.can_redo()); // 新しいストロークでredoは破棄
}

TEST_CASE("BrushStroke: 消しゴム") {
  PaintDocument d(100, 100);
  int l = d.add_layer("a");
  hline(d, l, black(10), 20);
  auto e   = black(10);
  e.eraser = true;
  hline(d, l, e, 20);
  CHECK(d.layers[l].img.get(50, 20)[3] == 0);
  CHECK(d.undo()); // 消しゴムをundoすると線が戻る
  CHECK(d.layers[l].img.get(50, 20)[3] == 255);
}

TEST_CASE("PaintDocument::composite: 不透明度・非表示・下地") {
  PaintDocument d(10, 10);
  int a = d.add_layer("a"), b = d.add_layer("b");
  d.layers[a].img.at(1, 1) = Vec4b{255, 0, 0, 255};
  d.layers[b].img.at(1, 1) = Vec4b{0, 0, 255, 255};
  d.layers[b].opacity      = 0.5f;
  PixelBuffer8 out;
  d.composite(out);
  CHECK(out(1, 1)[3] == 255);
  CHECK(out(1, 1)[0] == doctest::Approx(128).epsilon(0.02));
  CHECK(out(1, 1)[2] == doctest::Approx(128).epsilon(0.02));
  CHECK(out(0, 0)[3] == 0);
  d.layers[b].visible = false;
  d.composite(out, Vec4b{9, 9, 9, 255});
  CHECK(out(1, 1)[0] == 255);
  CHECK(out(0, 0)[0] == 9);

  d.layers[b].visible = true;
  d.layers[b].opacity = 1.0f;
  d.layers[b].blend   = Blend_Mul; // 赤 x 青 = 黒
  d.composite(out);
  CHECK(out(1, 1)[0] == 0);
  CHECK(out(1, 1)[2] == 0);
}

TEST_CASE("PaintDocument: .mup保存・読込のラウンドトリップと不正ファイル") {
  PaintDocument d(150, 90);
  int l = d.add_layer("線画");
  hline(d, l, black(6), 40);
  d.layers[l].opacity = 0.75f;
  const auto path     = (std::filesystem::temp_directory_path() / "paint_test.mup").string();
  REQUIRE(d.save(path));
  auto r = PaintDocument::load(path);
  REQUIRE(r);
  CHECK(r->width() == 150);
  CHECK(r->layers.size() == 1);
  CHECK(r->layers[0].name == "線画");
  CHECK(r->layers[0].opacity == 0.75f);
  PixelBuffer8 a, b;
  d.composite(a);
  r->composite(b);
  CHECK(std::memcmp(a.data(), b.data(), a.size_in_bytes()) == 0);

  // 先頭を壊すと読めない / 途中で切れていても落ちない
  {
    FILE* f = fopen(path.c_str(), "r+b");
    fputc('X', f);
    fclose(f);
  }
  CHECK(PaintDocument::load(path) == nullptr);
  CHECK(PaintDocument::load(path + ".none") == nullptr);
  std::filesystem::remove(path);
}
