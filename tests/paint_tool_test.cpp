#include "virtual_pen.hpp"
#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/app/app_impl.hpp>
#include <movutl/core/status_log.hpp>
#include <movutl/gui/paint/canvas_view.hpp>
#include <movutl/gui/paint/editor_state.hpp>
#include <movutl/input/mouse_pen_input.hpp>

using namespace mu;

namespace {
void clear_history() {
  while(undo_command()) {
  }
}
// 1ストローク分の入力
void stroke(PaintEditorState& s, float x0, float x1, float y, bool eraser = false) {
  MousePenInput in;
  in.update(x0, y, true, 0.0);
  in.update(x1, y, true, 0.1);
  in.update(x1, y, false, 0.2);
  for(auto e : in.poll()) {
    e.eraser = eraser;
    s.pointer(e);
  }
}
int alpha(const PaintEditorState& s, int x, int y) { return s.doc()->layers[s.active_layer()].img.get(x, y)[3]; }
std::string tmp_file(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }
} // namespace

TEST_CASE("MousePenInput: ボタンと移動からDown/Move/Upを作る") {
  MousePenInput in;
  in.update(1, 1, false, 0); // ホバーは出さない
  in.update(1, 1, true, 1);
  in.update(1, 1, true, 2); // 動かなければ出さない
  in.update(2, 3, true, 3);
  in.update(4, 5, false, 4);
  auto ev = in.poll();
  REQUIRE(ev.size() == 3);
  CHECK(ev[0].phase == PenPhase::Down);
  CHECK(ev[1].phase == PenPhase::Move);
  CHECK(ev[2].phase == PenPhase::Up);
  CHECK(ev[2].sample.x == 4);
  CHECK(ev[2].sample.pressure == 1.0f);
  CHECK(in.poll().empty());
}

TEST_CASE("PaintEdit: レイヤー操作を共通履歴でundo/redoできる") {
  clear_history();
  PaintDocument d(20, 20);
  d.add_layer("a");
  d.add_layer("b");
  auto names = [&] {
    std::string s;
    for(auto& L : d.layers) s += L.name;
    return s;
  };
  push_paint_edit(d.insert_layer(1, "c"));
  CHECK(names() == "acb");
  d.layers[1].img.at(3, 3) = Vec4b{1, 2, 3, 255};
  push_paint_edit(d.move_layer(0, 2));
  CHECK(names() == "cba");
  push_paint_edit(d.remove_layer(0));
  CHECK(names() == "ba");
  LayerProps p = d.layers[0];
  p.opacity    = 0.5f;
  p.blend      = Blend_Mul;
  push_paint_edit(d.set_layer_props(0, p));
  CHECK(d.layers[0].opacity == 0.5f);
  CHECK(d.set_layer_props(0, p) == nullptr); // 変化なし
  CHECK(d.remove_layer(5) == nullptr);

  const auto rev = d.revision();
  CHECK(undo_command());
  CHECK(d.revision() > rev);
  CHECK(d.layers[0].opacity == 1.0f);
  CHECK(d.layers[0].blend == Blend_Alpha);
  CHECK(undo_command());
  CHECK(names() == "cba");
  CHECK(d.layers[0].img.get(3, 3)[3] == 255); // 削除したレイヤーの画素も戻る
  CHECK(undo_command());
  CHECK(names() == "acb");
  CHECK(undo_command());
  CHECK(names() == "ab");
  CHECK_FALSE(can_undo());
  while(redo_command()) {
  }
  CHECK(names() == "ba");
  CHECK(d.layers[0].opacity == 0.5f);
}

TEST_CASE("PaintEdit: 破棄した文書の履歴は捨てられ、前の操作が取り消せる") {
  clear_history();
  PaintDocument keep(20, 20);
  keep.add_layer("a");
  push_paint_edit(keep.insert_layer(1, "b"));
  {
    PaintDocument gone(20, 20);
    gone.add_layer("a");
    push_paint_edit(gone.insert_layer(1, "b"));
  }
  CHECK(undo_command());
  CHECK(keep.layers.size() == 1);
  CHECK_FALSE(can_undo());
}

TEST_CASE("PaintDocument: ストローク中のundoはストロークを中止してから戻す") {
  clear_history();
  PaintDocument d(100, 100);
  d.add_layer("a");
  BrushSettings s;
  test::VirtualPen{}.draw(d, 0, s, test::VirtualPen::line(10, 20, 90, 20, 8));
  REQUIRE(d.begin_stroke(0, s));
  d.add_sample({50, 60, 1});
  CHECK(d.insert_layer(1, "x") == nullptr); // 描画中は構成を変えない
  CHECK(undo_command());
  CHECK_FALSE(d.stroking());
  CHECK(d.layers[0].img.tile_count() == 0);
  CHECK(d.end_stroke() == nullptr);
}

TEST_CASE("ToolManager: 同じキーで順送り、一時切替、操作中の切替") {
  ToolManager m;
  CHECK(m.active().id == "pen");
  CHECK(m.select_by_key("p"));
  CHECK(m.active().id == "pencil");
  CHECK(m.select_by_key("p"));
  CHECK(m.active().id == "pen");
  CHECK(m.select_by_key("b"));
  CHECK(m.active().id == "brush");
  CHECK(m.select_by_key("b"));
  CHECK(m.active().id == "airbrush");
  CHECK(m.select_by_key("e"));
  CHECK(m.select_by_key("b"));
  CHECK(m.active().id == "airbrush"); // そのキーで最後に使ったツール
  CHECK_FALSE(m.select_by_key("z"));
  CHECK_FALSE(m.select("nothing"));

  CHECK(m.begin_temporary("eyedropper"));
  CHECK(m.active().id == "eyedropper");
  CHECK(m.selected().id == "airbrush");
  m.end_temporary();
  CHECK(m.active().id == "airbrush");

  PaintDocument d(40, 40);
  d.add_layer("a");
  ToolContext c;
  c.doc = &d;
  REQUIRE(m.press(c, {20, 20, 1}));
  m.select_by_key("i"); // 途中で切り替えても、描き始めたツールが終える
  CHECK(m.release() != nullptr);
  CHECK_FALSE(m.busy());
}

TEST_CASE("PaintEditorState: 描く・透明色・ペンの消しゴム側・スポイト・undo") {
  clear_history();
  PaintEditorState s;
  CHECK_FALSE(s.pointer({{1, 1, 1}, PenPhase::Down}));
  CHECK_FALSE(s.new_document(0, 10));
  REQUIRE(s.new_document(100, 100));
  CHECK_FALSE(s.modified());

  s.set_color(Vec4b{200, 0, 0, 255});
  stroke(s, 10, 90, 20);
  CHECK(alpha(s, 50, 20) == 255);
  CHECK(alpha(s, 90, 20) == 255); // Upの位置まで描かれる
  CHECK(s.modified());
  CHECK(undo_command());
  CHECK(alpha(s, 50, 20) == 0);
  CHECK(redo_command());

  s.toggle_transparent();
  stroke(s, 40, 60, 20);
  CHECK(alpha(s, 50, 20) == 0);
  s.toggle_transparent();
  CHECK(s.color_slot == PaintColorSlot::Main);
  CHECK(undo_command());

  stroke(s, 40, 60, 20, true); // 消しゴム側
  CHECK(alpha(s, 50, 20) == 0);
  CHECK(s.tools.active().id == "pen");
  CHECK(undo_command());

  s.swap_color();
  CHECK(s.color() == s.sub_color);
  s.tools.select_by_key("i");
  CHECK(s.preset() == nullptr);
  CHECK_FALSE(s.step_brush_size(1));
  stroke(s, 50, 50, 20);
  CHECK(s.sub_color == Vec4b{200, 0, 0, 255});
  stroke(s, 50, 50, 80); // 透明な所は拾わない
  CHECK(s.sub_color == Vec4b{200, 0, 0, 255});

  // 描画中の破棄
  s.tools.select("pen");
  s.pointer({{10, 60, 1}, PenPhase::Down});
  s.cancel();
  CHECK(alpha(s, 10, 60) == 0);
}

TEST_CASE("PaintEditorState: レイヤー操作と保存・開き直し") {
  clear_history();
  PaintEditorState s;
  REQUIRE(s.new_document(64, 64));
  CHECK_FALSE(s.remove_layer()); // 最後の1枚
  CHECK(s.add_layer());
  CHECK(s.active_layer() == 1);
  stroke(s, 10, 50, 30);
  CHECK(s.move_layer(0));
  CHECK(alpha(s, 30, 30) == 255);
  CHECK(undo_command()); // 移動
  CHECK(undo_command()); // ストローク
  CHECK(undo_command()); // 追加
  CHECK(s.active_layer() == 0);
  CHECK(redo_command());
  CHECK(redo_command());

  CHECK_FALSE(s.save()); // 保存先が未定
  const auto path = tmp_file("paint_editor_test.mup");
  REQUIRE(s.save_as(path));
  CHECK_FALSE(s.modified());
  CHECK_FALSE(std::filesystem::exists(path + ".tmp"));
  stroke(s, 10, 50, 50);
  CHECK(s.modified());
  REQUIRE(s.open(path));
  CHECK(s.doc()->layers.size() == 2);
  CHECK(s.doc()->layers[1].img.get(30, 30)[3] == 255);
  CHECK_FALSE(s.open(path + ".none"));
  CHECK(s.path() == path);
  CHECK_FALSE(can_undo()); // 閉じた文書の履歴は残らない
  std::filesystem::remove(path);
}

TEST_CASE("BrushLibrary: 標準サブツール、初期値に戻す、JSON保存と読込") {
  auto lib = BrushLibrary::standard();
  ToolManager m;
  for(const auto& t : m.tools())
    if(t->uses_brush()) CHECK_MESSAGE(lib.selected(t->id), t->id);
  for(const char* engine : {"paint", "erase", "airbrush", "watercolor", "tone", "particle", "pattern"}) {
    bool found = false;
    for(const auto& p : lib.presets) found |= p.initial.engine == engine;
    CHECK_MESSAGE(found, engine);
  }
  REQUIRE(lib.select("deco", "パターン"));
  CHECK(lib.selected("deco")->current.pattern);
  CHECK_FALSE(lib.select("deco", "Gペン"));

  auto* pen         = lib.selected("pen");
  pen->current.size = 33;
  pen->current.params.set<int32_t>("arm", 7);
  pen->current.stabilization = 40;
  const auto path            = tmp_file("paint_brushes_test.json");
  REQUIRE(lib.save(path));
  BrushLibrary r;
  REQUIRE(r.load(path));
  CHECK(r.presets.size() == lib.presets.size());
  CHECK(r.selected("deco")->name == "パターン");
  CHECK(r.selected("deco")->current.pattern);
  CHECK(r.selected("pen")->current.size == 33);
  CHECK(r.selected("pen")->current.stabilization == 40);
  CHECK(r.selected("pen")->initial.size == 10);
  CHECK(cutil::get_or<int32_t>(r.selected("pen")->current.params, "arm", 0) == 7);
  r.selected("pen")->reset();
  CHECK(r.selected("pen")->current.size == 10);

  for(const char* bad : {"{", "{}", R"({"presets":[{"name":1}]})"}) {
    std::ofstream(path) << bad;
    CHECK_FALSE(r.load(path));
    CHECK(r.presets.size() == lib.presets.size()); // 失敗しても内容は変わらない
  }
  std::filesystem::remove(path);
}

TEST_CASE("paintコマンド: ツール切替・ブラシサイズ・不透明度・描画色と、キーの衝突解消") {
  detail::register_default_commands();
  auto& s = paint_editor();
  s.tools.select("pen");
  CHECK(run_command("paint_tool_pen"));
  CHECK(s.tools.active().id == "pencil");
  CHECK(run_command("paint_tool_pen"));

  auto& cur = s.preset()->current;
  cur.size  = 10;
  CHECK(run_command("paint_brush_size_up"));
  CHECK(cur.size == 12);
  CHECK(run_command("paint_brush_size_down"));
  CHECK(cur.size == 10);
  cur.size = 1;
  CHECK(run_command("paint_brush_size_down"));
  CHECK(cur.size == 1);
  cur.opacity = 1;
  CHECK(run_command("paint_opacity_down"));
  CHECK(cur.opacity == doctest::Approx(0.9f));
  for(int i = 0; i < 20; i++) run_command("paint_opacity_down");
  CHECK(cur.opacity == doctest::Approx(0.01f));
  CHECK(run_command("paint_opacity_up"));
  CHECK(cur.opacity == doctest::Approx(0.1f));
  s.preset()->reset();

  CHECK(run_command("paint_swap_color"));
  CHECK(s.color_slot == PaintColorSlot::Sub);
  CHECK(run_command("paint_transparent_color"));
  CHECK(s.brush_settings().engine == "erase");
  CHECK(run_command("paint_transparent_color"));
  CHECK(s.color_slot == PaintColorSlot::Sub);
  CHECK(run_command("paint_swap_color"));

  std::map<std::string, std::string> keys; // key → id
  for(const auto& i : get_command_infos()) {
    if(i.id.rfind("paint_", 0) != 0 && i.id != "toggle_keyframe" && i.id != "import_exo") continue;
    if(i.id == "import_exo") CHECK(i.shortcut.empty());
    if(i.shortcut.empty()) continue;
    CHECK_MESSAGE(keys.emplace(i.shortcut, i.id).second, i.shortcut);
    if(i.id.rfind("paint_brush", 0) == 0) CHECK(i.context == "canvas");
  }
  CHECK(keys["p"] == "paint_tool_pen");
  CHECK(keys["shift+p"] == "toggle_keyframe");
}

TEST_CASE("CanvasView: screenとcanvasの往復、拡大縮小・回転・左右反転") {
  CanvasView v;
  v.view_pos    = Vec2(100, 50);
  v.view_size   = Vec2(800, 600);
  v.canvas_size = Vec2(400, 200);
  auto near     = [](const Vec2& a, const Vec2& b) { return std::abs(a[0] - b[0]) < 0.01f && std::abs(a[1] - b[1]) < 0.01f; };

  CHECK(near(v.to_screen(Vec2(200, 100)), Vec2(500, 350))); // キャンバス中心はビュー中心
  CHECK(near(v.to_screen(Vec2(0, 0)), Vec2(300, 250)));

  v.fit();
  CHECK(v.zoom == doctest::Approx(2.0f));

  const Vec2 pivot(620, 180);
  const Vec2 under = v.to_canvas(pivot);
  v.zoom_at(pivot, 1.7f);
  CHECK(near(v.to_canvas(pivot), under));
  v.rotate_at(pivot, 0.6f);
  CHECK(near(v.to_canvas(pivot), under));
  v.move(Vec2(13, -7));

  const Vec2 before = v.to_screen(Vec2(30, 40));
  v.flip_horizontal();
  CHECK(near(v.to_screen(Vec2(30, 40)), Vec2(1000 - before[0], before[1]))); // ビュー中心x=500で鏡映

  for(const Vec2& c : {Vec2(0, 0), Vec2(30, 40), Vec2(399.5f, 12.25f)}) CHECK(near(v.to_canvas(v.to_screen(c)), c));

  v.angle = 3.14159265f / 2; // 90度回すと縦横が入れ替わる
  v.fit();
  CHECK(v.zoom == doctest::Approx(1.5f));
  v.zoom_at(pivot, 1e9f);
  CHECK(v.zoom == CanvasView::kMaxZoom);
}

TEST_CASE("PaintEditorState: 画素が変わらない操作は未保存にしない") {
  clear_history();
  status_log_set_dirty(false);
  PaintEditorState s;
  REQUIRE(s.new_document(100, 100));
  const auto rev = s.doc()->revision();

  stroke(s, 300, 400, 300); // キャンバス外
  s.pointer({{10, 60, 1}, PenPhase::Down});
  s.cancel(); // Esc
  s.toggle_transparent();
  stroke(s, 10, 90, 20); // 何も無い所を消す
  s.toggle_transparent();
  CHECK_FALSE(s.modified());
  CHECK(s.doc()->revision() == rev);
  CHECK(s.doc()->layers[0].img.tile_count() == 0);
  CHECK_FALSE(can_undo());

  // ストローク中の他の変更は、取り消しても残る
  s.pointer({{10, 60, 1}, PenPhase::Down});
  LayerProps p = s.doc()->layers[0];
  p.opacity    = 0.5f;
  CHECK(s.set_layer_props(0, p));
  s.cancel();
  CHECK(s.modified());

  // ペイントの操作は動画プロジェクトを未保存にしない
  stroke(s, 10, 90, 20);
  CHECK(undo_command());
  CHECK(redo_command());
  CHECK_FALSE(status_log_dirty());
  clear_history();
  status_log_set_dirty(false);
}
