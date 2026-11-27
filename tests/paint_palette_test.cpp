// 基本パレットが使う処理(サブツールの編集、ブラシ設定の保存、カラーヒストリー)。ユーザーのファイルは触らない
#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/core/command.hpp>
#include <movutl/gui/paint/palettes.hpp>

using namespace mu;

namespace {
std::string names(BrushLibrary& lib, const char* tool) {
  std::string s;
  for(auto* p : lib.of_tool(tool)) s += p->name + "/";
  return s;
}
} // namespace

TEST_CASE("BrushLibrary: サブツールの複製・削除・並べ替え") {
  auto lib = BrushLibrary::standard();
  CHECK_FALSE(lib.remove("pen", "Gペン")); // ツール最後の1つは消せない
  lib.selected("pen")->current.size = 33;
  REQUIRE(lib.duplicate("pen", "Gペン"));
  CHECK(names(lib, "pen") == "Gペン/Gペン のコピー/");
  CHECK(lib.selected("pen")->name == "Gペン のコピー");
  CHECK(lib.selected("pen")->current.size == 33);
  REQUIRE(lib.duplicate("pen", "Gペン"));
  CHECK(names(lib, "pen") == "Gペン/Gペン のコピー 2/Gペン のコピー/");

  REQUIRE(lib.move("pen", "Gペン", +1));
  CHECK(names(lib, "pen") == "Gペン のコピー 2/Gペン/Gペン のコピー/");
  CHECK(lib.selected("pen")->name == "Gペン のコピー 2"); // 選択は名前で保たれる
  CHECK_FALSE(lib.move("pen", "Gペン のコピー 2", -1));
  // 別のグループとは入れ替えない
  CHECK_FALSE(lib.move("deco", "トーン", +1));

  REQUIRE(lib.remove("pen", "Gペン のコピー 2"));
  CHECK(lib.selected("pen")->name == "Gペン");
  CHECK_FALSE(lib.remove("pen", "無いもの"));

  // 未選択(先頭)のサブツールを動かしても選択は同じもののまま
  auto fresh = BrushLibrary::standard();
  fresh.duplicate("eraser", "硬め");
  auto reloaded    = BrushLibrary::standard();
  reloaded.presets = fresh.presets;
  REQUIRE(reloaded.move("eraser", "硬め", +1));
  CHECK(reloaded.selected("eraser")->name == "硬め");
}

TEST_CASE("BrushStore: 変更があれば保存し、起動時に読み込む。壊れたファイルは退避する") {
  const auto dir = std::filesystem::temp_directory_path() / "mu_brush_store_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const auto path = (dir / "brushes.json").string();

  auto lib = BrushLibrary::standard();
  BrushStore store{path, {}};
  store.load(lib);
  CHECK_FALSE(store.flush(lib)); // 変更が無ければファイルを作らない
  CHECK_FALSE(std::filesystem::exists(path));
  lib.selected("pen")->current.size = 42;
  lib.duplicate("pen", "Gペン");
  CHECK(store.flush(lib));
  CHECK_FALSE(store.flush(lib));

  auto next = BrushLibrary::standard();
  BrushStore store2{path, {}};
  store2.load(next);
  CHECK(next.of_tool("pen").size() == 2);
  CHECK(next.selected("pen")->name == "Gペン のコピー");
  CHECK(next.selected("pen")->current.size == 42);

  std::ofstream(path) << "{broken";
  auto fallback = BrushLibrary::standard();
  BrushStore store3{path, {}};
  store3.load(fallback);
  CHECK(std::filesystem::exists(path + ".bak"));
  CHECK_FALSE(std::filesystem::exists(path));
  std::filesystem::remove_all(dir);
}

TEST_CASE("PaintEditorState: 描画に使った色をカラーヒストリーに残す") {
  PaintEditorState s;
  REQUIRE(s.new_document(32, 32));
  const auto draw = [&] {
    s.pointer({{4, 4, 1}, PenPhase::Down});
    s.pointer({{8, 8, 1}, PenPhase::Up});
  };
  CHECK(s.color_history.empty());
  s.set_color(Vec4b{255, 0, 0, 255});
  draw();
  s.set_color(Vec4b{0, 255, 0, 255});
  draw();
  s.set_color(Vec4b{255, 0, 0, 255});
  draw();
  REQUIRE(s.color_history.size() == 2); // 重複はまとめて先頭へ
  CHECK(s.color_history[0][0] == 255);
  s.toggle_transparent();
  s.sub_color = Vec4b{9, 9, 9, 255};
  draw();
  CHECK(s.color_history.size() == 2); // 透明色(消しゴム)は残さない
  while(undo_command()) {
  }
}

TEST_CASE("PaintDocument: ストローク中でもレイヤーの属性は変えられる(名前の確定がキャンバスのクリックより後になる)") {
  PaintDocument d(16, 16);
  d.add_layer("a");
  REQUIRE(d.begin_stroke(0, BrushSettings()));
  d.add_sample({4, 4, 1});
  LayerProps p = d.layers[0];
  p.name       = "b";
  auto rename  = d.set_layer_props(0, p);
  REQUIRE(rename);
  CHECK(d.stroking());
  auto stroke = d.end_stroke();
  REQUIRE(stroke);
  stroke->undo();
  rename->undo();
  CHECK(d.layers[0].name == "a");
  CHECK(d.layers[0].img.get(4, 4)[3] == 0);
}
