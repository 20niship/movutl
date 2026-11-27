#include <doctest/doctest.h>
#include <movutl/app/paint_commands.hpp>
#include <movutl/gui/paint/editor_state.hpp>
#include <movutl/input/xinput2_pen_input.hpp>
#include <movutl/paint/stabilizer.hpp>

using namespace mu;

TEST_CASE("Stabilizer: 強さ0は素通し") {
  Stabilizer s(0);
  for(const PenSample p : {PenSample{1.25f, 2, 0.3f}, PenSample{100, -5, 0.9f}, PenSample{3, 3, 1}}) {
    const auto o = s.add(p);
    CHECK(o.x == p.x);
    CHECK(o.y == p.y);
    CHECK(o.pressure == p.pressure);
  }
  CHECK(s.finish().empty());
}

TEST_CASE("Stabilizer: ブレを抑え、点を欠かさず、終点まで追いつく") {
  CHECK(Stabilizer().finish().empty()); // 入力無し
  Stabilizer s(20);                     // 窓は11点
  PenSample o = s.add({0, 0, 0.5f});
  CHECK(o.x == 0);
  float amp = 0;
  for(int i = 1; i <= 40; i++) {
    o = s.add({(float)i, i % 2 ? 5.0f : -5.0f, i / 40.0f});
    CHECK(o.pressure == i / 40.0f);
    if(i > 11) amp = std::max(amp, std::abs(o.y));
  }
  CHECK(amp < 1.0f);
  CHECK(o.x < 36); // 遅れている
  const auto tail = s.finish();
  REQUIRE(tail.size() == 10);
  float prev = o.x;
  for(const auto& p : tail) {
    CHECK(p.x >= prev);
    CHECK(p.pressure == 1.0f);
    prev = p.x;
  }
  CHECK(tail.back().x == 40);
  CHECK(tail.back().y == -5);
  CHECK(Stabilizer(1000).add({7, 8, 1}).x == 7); // 範囲外の強さ
}

TEST_CASE("PaintEditorState: 手ブレ補正を掛けても終点まで描かれる") {
  PaintEditorState s;
  REQUIRE(s.new_document(100, 100));
  s.preset()->current.stabilization = 100;
  s.pointer({{10, 20, 1}, PenPhase::Down});
  s.pointer({{50, 20, 1}, PenPhase::Move});
  auto alpha = [&](int x, int y) { return s.doc()->layers[0].img.get(x, y)[3]; };
  CHECK(alpha(40, 20) == 0); // 補正で遅れる
  s.pointer({{90, 20, 1}, PenPhase::Up});
  for(int x = 10; x <= 90; x++) CHECK(alpha(x, 20) == 255);
  while(undo_command()) {
  }
}

#ifdef __linux__

namespace {
// 記録した入力の再生: 軸0=X, 1=Y, 2=Pressure(0〜2047)。maskで立つ軸の値だけがvaluesに並ぶ
struct Record {
  PenPhase phase;
  int button;
  double x, y;
  unsigned char mask;
  double values[3];
};
std::vector<PenEvent> replay(xi2::PenDevice& d, const std::vector<Record>& records) {
  std::vector<PenEvent> out;
  double t = 0;
  for(const auto& r : records)
    if(auto e = xi2::translate(d, {r.phase, r.button, r.x, r.y, t += 0.005, &r.mask, 1, r.values})) out.push_back(*e);
  return out;
}
} // namespace

TEST_CASE("xi2: valuatorの読み取り、筆圧の正規化、消しゴム側の判別") {
  const unsigned char mask[2] = {0b00000101, 0b00000010}; // 軸0, 2, 9
  const double values[]       = {10, 20, 30};
  CHECK(xi2::read_valuator(mask, 2, values, 0) == 10);
  CHECK(xi2::read_valuator(mask, 2, values, 2) == 20);
  CHECK(xi2::read_valuator(mask, 2, values, 9) == 30);
  CHECK_FALSE(xi2::read_valuator(mask, 2, values, 1));
  CHECK_FALSE(xi2::read_valuator(mask, 1, values, 9)); // maskの外
  CHECK_FALSE(xi2::read_valuator(mask, 2, values, -1));

  CHECK(xi2::normalize_pressure(0, 0, 2047) == 0.0f);
  CHECK(xi2::normalize_pressure(2047, 0, 2047) == 1.0f);
  CHECK(xi2::normalize_pressure(9999, 0, 2047) == 1.0f);
  CHECK(xi2::normalize_pressure(-1, 0, 2047) == 0.0f);
  CHECK(xi2::normalize_pressure(5, 0, 0) == 1.0f);

  CHECK(xi2::is_eraser_name("Wacom Intuos S Pen eraser"));
  CHECK(xi2::is_eraser_name("UGTABLET 10 inch PenTablet Pen Eraser (0)"));
  CHECK(xi2::is_eraser_name("xwayland-tablet eraser:12"));
  CHECK_FALSE(xi2::is_eraser_name("UGTABLET 10 inch PenTablet Pen Pen (0)"));
}

TEST_CASE("xi2: 記録した入力の再生(ホバーとサイドボタンは出さず、接地中は全点を返す)") {
  xi2::PenDevice d;
  d.pressure_axis = 2;
  d.pressure_max  = 2047;
  const auto ev   = replay(d, {
                                {PenPhase::Move, 0, 5, 5, 0b111, {100, 100, 0}},      // ホバー
                                {PenPhase::Down, 3, 5, 5, 0, {}},                     // サイドボタン
                                {PenPhase::Up, 3, 5, 5, 0, {}},                       //
                                {PenPhase::Down, 1, 10.25, 20.5, 0b100, {1023.5}},    // 筆圧だけ
                                {PenPhase::Move, 0, 11.75, 20.5, 0b001, {200}},       // 筆圧が無い → 直前の値
                                {PenPhase::Move, 0, 13, 21, 0b111, {300, 100, 2047}}, //
                                {PenPhase::Up, 1, 13, 21, 0b100, {0}},                // 離した時の0は使わない
                                {PenPhase::Move, 0, 14, 21, 0b111, {400, 100, 0}},    // ホバー
                                {PenPhase::Up, 1, 14, 21, 0, {}},                     // Downの無いUp
                              });
  REQUIRE(ev.size() == 4);
  CHECK(ev[0].phase == PenPhase::Down);
  CHECK(ev[0].sample.x == 10.25f);
  CHECK(ev[0].sample.y == 20.5f);
  CHECK(ev[0].sample.pressure == doctest::Approx(0.5f));
  CHECK(ev[1].sample.x == 11.75f);
  CHECK(ev[1].sample.pressure == doctest::Approx(0.5f));
  CHECK(ev[2].sample.pressure == 1.0f);
  CHECK(ev[3].phase == PenPhase::Up);
  CHECK(ev[3].sample.pressure == 1.0f);
  CHECK_FALSE(ev[3].eraser);
  CHECK(ev[1].time > ev[0].time);
  CHECK_FALSE(d.down);
}

TEST_CASE("xi2 -> PaintEditorState: 消しゴム側の間だけ消しゴムになり、ペン先に戻すと元のツール") {
  PaintEditorState s;
  REQUIRE(s.new_document(100, 100));
  s.tools.select("pencil");
  xi2::PenDevice tip, eraser;
  tip.pressure_axis = eraser.pressure_axis = 2;
  tip.pressure_max = eraser.pressure_max = 2047;
  eraser.eraser                          = true;
  auto line                              = [](double y) { return std::vector<Record>{{PenPhase::Down, 1, 10, y, 0b100, {2047}}, {PenPhase::Move, 0, 90, y, 0b100, {2047}}, {PenPhase::Up, 1, 90, y, 0, {}}}; };
  auto alpha                             = [&] { return s.doc()->layers[0].img.get(50, 20)[3]; };

  for(const auto& e : replay(tip, line(20))) s.pointer(e);
  const int drawn = alpha();
  CHECK(drawn > 0);

  for(const auto& e : replay(eraser, line(20))) {
    s.pointer(e);
    if(e.phase != PenPhase::Up) CHECK(s.tools.active().id == "eraser");
  }
  CHECK(alpha() == 0);
  CHECK(s.tools.active().id == "pencil");
  CHECK_FALSE(s.tools.temporary());

  for(const auto& e : replay(tip, line(20))) {
    s.pointer(e);
    CHECK(s.tools.active().id == "pencil");
  }
  CHECK(alpha() == drawn);

  // 消しゴム側のストロークを破棄した場合も、Upを取りこぼして次がペン先の場合も戻る
  s.pointer({{10, 20, 1}, PenPhase::Down, true});
  s.cancel();
  CHECK(s.tools.active().id == "pencil");
  s.pointer({{10, 40, 1}, PenPhase::Down, true});
  s.pointer({{10, 60, 1}, PenPhase::Down, false});
  CHECK(s.tools.active().id == "pencil");
  s.cancel();
  while(undo_command()) {
  }
}

TEST_CASE("XInput2PenInput: 開けない場合はfalseで、何も返さない") {
  XInput2PenInput in;
  CHECK_FALSE(in.open(0)); // Xが無いか、無効なwindow
  CHECK_FALSE(in.has_pen());
  CHECK_FALSE(in.pen_active());
  CHECK(in.poll().empty());
}

#endif
