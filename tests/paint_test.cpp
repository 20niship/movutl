#include "virtual_pen.hpp"
#include <doctest/doctest.h>
#include <filesystem>
#include <movutl/core/opencv_compat.hpp>
#include <movutl/paint/paint_document.hpp>
#include <movutl/paint/stroke_registry.hpp>
#include <movutl/paint/strokes.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;
using mu::test::VirtualPen;

namespace {
BrushSettings black(float size) {
  BrushSettings s;
  s.size     = size;
  s.hardness = 1.0f;
  s.color    = Vec4b{0, 0, 0, 255};
  return s;
}
void hline(PaintDocument& d, int layer, const BrushSettings& s, float y, float p = 1.0f) {
  VirtualPen{}.draw(d, layer, s, VirtualPen::line(10, y, 90, y, 16, [](float) { return 1.0f; }));
  (void)p;
}
std::shared_ptr<PixelBuffer8> asset(const char* name) {
  auto img = load_brush_image(std::string("../assets/brushes/") + name);
  REQUIRE_MESSAGE(img, "assets/brushes/" << name << " が読めない(assets submoduleを更新したか)");
  return img;
}
int alpha_sum(const TiledImage& t) {
  int n = 0;
  for(int y = 0; y < t.height(); y++)
    for(int x = 0; x < t.width(); x++) n += t.get(x, y)[3];
  return n;
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

TEST_CASE("paint: 太さと筆圧") {
  PaintDocument d(100, 100);
  int l = d.add_layer("a");
  hline(d, l, black(10), 20);
  CHECK(d.layers[l].img.get(50, 20)[3] == 255);
  CHECK(d.layers[l].img.get(50, 26)[3] == 0); // 半径5の外

  int l2 = d.add_layer("b");
  VirtualPen{}.draw(d, l2, black(10), VirtualPen::line(10, 50, 90, 50, 16, [](float) { return 0.3f; })); // 筆圧0.3 → 直径3
  CHECK(d.layers[l2].img.get(50, 50)[3] > 200);
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

TEST_CASE("erase: 塗った所だけ消え、undoで戻る") {
  PaintDocument d(100, 100);
  int l = d.add_layer("a");
  hline(d, l, black(10), 20);
  auto e   = black(10);
  e.engine = "erase";
  hline(d, l, e, 20);
  CHECK(d.layers[l].img.get(50, 20)[3] == 0);
  CHECK(d.undo());
  CHECK(d.layers[l].img.get(50, 20)[3] == 255);
}

TEST_CASE("PaintDocument::composite: 不透明度・非表示・下地・合成モード") {
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

  { // 先頭を壊すと読めない / 存在しないファイルでも落ちない
    FILE* f = fopen(path.c_str(), "r+b");
    fputc('X', f);
    fclose(f);
  }
  CHECK(PaintDocument::load(path) == nullptr);
  CHECK(PaintDocument::load(path + ".none") == nullptr);
  std::filesystem::remove(path);
}

TEST_CASE("ImageTip: 黒=描く、白=描かない。アルファの外は描かない") {
  auto tip = std::make_shared<PixelBuffer8>(4, 4);
  tip->fill(Vec4b{0, 0, 0, 255});
  (*tip)(0, 0) = Vec4b{255, 255, 255, 255}; // 左上は白=描かない
  PaintDocument d(60, 60);
  int l           = d.add_layer("a");
  auto s          = black(20);
  s.tip           = std::make_shared<ImageTip>(tip);
  s.pressure_size = false;
  VirtualPen{}.draw(d, l, s, {{30, 30, 1}});
  CHECK(d.layers[l].img.get(35, 35)[3] == 255); // 先端の内側
  CHECK(d.layers[l].img.get(21, 21)[3] == 0);   // 白い左上
  CHECK(d.layers[l].img.get(45, 30)[3] == 0);   // 先端の外
}

TEST_CASE("BrushTexture: 紙目の暗い所は乗らず、depth=0で無効") {
  auto tex     = std::make_shared<PixelBuffer8>(2, 1);
  (*tex)(0, 0) = Vec4b{255, 255, 255, 255};
  (*tex)(1, 0) = Vec4b{0, 0, 0, 255};
  auto run     = [&](float depth) {
    PaintDocument d(40, 40);
    int l           = d.add_layer("a");
    auto s          = black(30);
    s.texture.img   = tex;
    s.texture.depth = depth;
    VirtualPen{}.draw(d, l, s, {{20, 20, 1}});
    return std::make_pair(d.layers[l].img.get(20, 20)[3], d.layers[l].img.get(21, 20)[3]); // 偶数x=白, 奇数x=黒
  };
  auto [w, b] = run(1.0f);
  CHECK(w == 255);
  CHECK(b == 0);
  auto [w0, b0] = run(0.0f);
  CHECK(b0 == 255);
}

TEST_CASE("airbrush: ペンを止めている間に濃くなる。paintは変わらない") {
  auto run = [&](const char* engine) {
    PaintDocument d(60, 60);
    int l           = d.add_layer("a");
    auto s          = black(20);
    s.engine        = engine;
    s.opacity       = 0.2f;
    s.airbrush_rate = 8.0f;
    VirtualPen{}.draw(d, l, s, {{30, 30, 1}}, 1.0f);
    return d.layers[l].img.get(30, 30)[3];
  };
  CHECK(run("airbrush") > run("paint") + 100);
}

TEST_CASE("tone: 濃さが網点の面積で表現され、値は2値のみ") {
  PaintDocument d(100, 40);
  int l              = d.add_layer("a");
  auto s             = black(30);
  s.engine           = "tone";
  s.pressure_opacity = true;
  s.pressure_size    = false;
  s.tone_pitch       = 5;
  VirtualPen{}.draw(d, l, s, VirtualPen::line(15, 20, 85, 20, 70, VirtualPen::ramp_up));
  int on_left = 0, on_right = 0;
  for(int y = 8; y < 32; y++) {
    for(int x = 0; x < 100; x++) {
      const int a = d.layers[l].img.get(x, y)[3];
      CHECK((a == 0 || a == 255));
      if(a == 255) (x < 50 ? on_left : on_right)++;
    }
  }
  CHECK(on_right > on_left);
}

TEST_CASE("particle: 同じseedで同じ結果、粒が撒かれる") {
  auto run = [&](uint32_t seed) {
    PaintDocument d(80, 80);
    int l     = d.add_layer("a");
    auto s    = black(40);
    s.engine  = "particle";
    s.seed    = seed;
    s.spacing = 0.5f;
    VirtualPen{}.draw(d, l, s, VirtualPen::line(20, 40, 60, 40, 8));
    PixelBuffer8 out;
    d.composite(out);
    return out;
  };
  auto a = run(1), b = run(1), c = run(2);
  CHECK(std::memcmp(a.data(), b.data(), a.size_in_bytes()) == 0);
  CHECK(std::memcmp(a.data(), c.data(), a.size_in_bytes()) != 0);
}

TEST_CASE("watercolor: 下地と混ざり、ストローク終了時に確定する") {
  PaintDocument d(60, 60);
  int l     = d.add_layer("a");
  auto red  = black(40);
  red.color = Vec4b{255, 0, 0, 255};
  VirtualPen{}.draw(d, l, red, {{30, 30, 1}});
  auto s      = black(20);
  s.engine    = "watercolor";
  s.color     = Vec4b{0, 0, 255, 255};
  s.water_mix = 0.5f;
  d.begin_stroke(l, s);
  d.add_sample({30, 30, 1});
  CHECK(d.layers[l].img.get(30, 30)[2] == 0); // end()前はまだ下地のまま
  d.end_stroke();
  const Vec4b c = d.layers[l].img.get(30, 30);
  CHECK(c[0] > 80);
  CHECK(c[2] > 80); // 赤と青が混ざって紫寄り
  CHECK(d.undo());  // 水彩もundoで戻る
  CHECK(d.layers[l].img.get(30, 30)[2] == 0);
}

TEST_CASE("pattern: キャンバス座標に固定されたパターンが貼られる") {
  auto pat     = std::make_shared<PixelBuffer8>(2, 1);
  (*pat)(0, 0) = Vec4b{255, 0, 0, 255};
  (*pat)(1, 0) = Vec4b{0, 255, 0, 255};
  PaintDocument d(40, 40);
  int l     = d.add_layer("a");
  auto s    = black(30);
  s.engine  = "pattern";
  s.pattern = pat;
  VirtualPen{}.draw(d, l, s, {{20, 20, 1}});
  CHECK(d.layers[l].img.get(20, 20)[0] == 255);
  CHECK(d.layers[l].img.get(21, 20)[1] == 255);
}

TEST_CASE("Selection: 選択範囲の外には全ての描画方式で書き込まれない") {
  for(const char* engine : {"paint", "erase", "airbrush", "pattern", "tone", "particle", "watercolor"}) {
    PaintDocument d(80, 80);
    int l                    = d.add_layer("a");
    d.layers[l].img.at(5, 5) = Vec4b{0, 0, 0, 255}; // 選択外にある既存の画素(消しゴムが触っても消えない)
    auto sel                 = Selection::rect(80, 80, 40, 0, 80, 80);
    d.set_clip(&sel);
    auto s       = black(40);
    s.engine     = engine;
    auto pat     = std::make_shared<PixelBuffer8>(1, 1);
    (*pat)(0, 0) = Vec4b{0, 0, 0, 255};
    s.pattern    = pat;
    VirtualPen{}.draw(d, l, s, VirtualPen::line(20, 40, 60, 40, 20), 0.2f);
    bool outside = false, inside = false;
    for(int y = 0; y < 80; y++)
      for(int x = 0; x < 80; x++) {
        if(x == 5 && y == 5) continue;
        if(d.layers[l].img.get(x, y)[3] > 0) (x < 40 ? outside : inside) = true;
      }
    CAPTURE(engine);
    CHECK_FALSE(outside);
    if(std::string(engine) != "erase") CHECK(inside);
    CHECK(d.layers[l].img.get(5, 5)[3] == 255);
  }
}

namespace {
// 任意のサブクラスの例: 触れた画素を全て不透明な赤にする
class RedStroke : public Stroke {
public:
  using Stroke::Stroke;
  void add(const PenSample& p) override {
    const int x = (int)p.x, y = (int)p.y;
    if(target_->in_range(x, y)) target_->at(x, y) = Vec4b{255, 0, 0, 255};
  }
};
} // namespace

TEST_CASE("StrokeRegistry: 自作のStroke派生クラスを登録して使える。未登録engineは失敗") {
  StrokePluginTable t;
  std::strncpy(t.name, "test_red", sizeof(t.name) - 1);
  t.create = [](TiledImage* i, const BrushSettings& s) -> Stroke* { return new RedStroke(i, s); };
  REQUIRE(StrokeRegistry::Get().register_type(t));

  PaintDocument d(10, 10);
  int l    = d.add_layer("a");
  auto s   = black(5);
  s.engine = "test_red";
  VirtualPen{}.draw(d, l, s, {{3, 4, 1}});
  CHECK(d.layers[l].img.get(3, 4)[0] == 255);
  CHECK(d.undo()); // 自作ブラシでもundoできる
  CHECK(d.layers[l].img.get(3, 4)[3] == 0);

  s.engine = "no_such_brush";
  CHECK_FALSE(d.begin_stroke(l, s));
  CHECK_FALSE(StrokeRegistry::Get().register_type(StrokePluginTable{})); // 名前が空
}

TEST_CASE("ブラシプラグイン: brush_sample.msoを読み込むと\"cross\"が登録され描ける") {
  detail::init_external_plugins();
  auto names = StrokeRegistry::Get().list();
  REQUIRE(std::find(names.begin(), names.end(), "cross") != names.end());
  PaintDocument d(30, 30);
  int l    = d.add_layer("a");
  auto s   = black(5);
  s.engine = "cross";
  cutil::Prop params;
  params.set<int32_t>("arm", 4);
  s.params = params;
  VirtualPen{}.draw(d, l, s, {{15, 15, 1}});
  CHECK(d.layers[l].img.get(15, 15)[3] == 255);
  CHECK(d.layers[l].img.get(19, 15)[3] == 255); // 腕4px
  CHECK(d.layers[l].img.get(20, 15)[3] == 0);
  CHECK(d.layers[l].img.get(17, 17)[3] == 0); // 斜めは塗らない
}

// ---- 仮想ペンで色々なブラシを動かして、見られる画像に書き出す ----
TEST_CASE("仮想ペンギャラリー: 各ブラシで描いてbuild/paint_gallery/gallery.pngに書き出す") {
  detail::init_external_plugins();
  constexpr int W = 1000, H = 1250, ROW = 70;
  PaintDocument d(W, H);
  int paper = d.add_layer("下書き"), ink = d.add_layer("ペン");
  VirtualPen pen;
  auto at   = [&](int row) { return ROW * row + ROW / 2.0f; };
  auto wave = [&](int row, VirtualPen::Pressure p = VirtualPen::bell) { return VirtualPen::wave(40, W - 40, at(row), 14, 2.5f, 160, p); };

  auto grain = asset("paper_grain.png");
  auto weave = asset("canvas_weave.png");

  int row = 0;
  { // 0: 硬い円(筆圧なし) / 1: 柔らかい円(筆圧で太さ)
    auto s          = black(14);
    s.pressure_size = false;
    pen.draw(d, ink, s, wave(row++, VirtualPen::full));
    auto t     = black(30);
    t.hardness = 0.1f;
    pen.draw(d, ink, t, wave(row++));
  }
  { // 2: チョーク先端 / 3: 平筆(進行方向に向ける) / 4: 星のスタンプ(散布+サイズばらつき)
    auto s    = black(26);
    s.tip     = std::make_shared<ImageTip>(asset("tip_chalk.png"));
    s.spacing = 0.15f;
    pen.draw(d, ink, s, wave(row++));
    auto t          = black(30);
    t.tip           = std::make_shared<ImageTip>(asset("tip_flat.png"));
    t.angle         = 0.9f;
    t.spacing       = 0.05f;
    t.pressure_size = false; // 固定角の平筆(カリグラフィ)
    pen.draw(d, ink, t, wave(row++, VirtualPen::full));
    auto u         = black(22);
    u.tip          = std::make_shared<ImageTip>(asset("tip_star.png"));
    u.color        = Vec4b{200, 120, 0, 255};
    u.spacing      = 0.9f;
    u.scatter      = 0.25f;
    u.size_jitter  = 0.5f;
    u.angle_follow = true;
    pen.draw(d, ink, u, wave(row++, VirtualPen::full));
  }
  { // 5: 毛筆先端 + 紙目 / 6: 鉛筆(硬い円 + 荒い紙目)
    auto s         = black(30);
    s.tip          = std::make_shared<ImageTip>(asset("tip_bristle.png"));
    s.texture      = {grain, 1.0f, 0.9f};
    s.angle_follow = true;
    s.spacing      = 0.06f;
    pen.draw(d, ink, s, wave(row++));
    auto t    = black(10);
    t.color   = Vec4b{60, 60, 60, 255};
    t.texture = {weave, 1.5f, 1.0f};
    t.opacity = 0.9f;
    pen.draw(d, ink, t, wave(row++));
  }
  { // 7: エアブラシ(途中で2回止めて濃さを積む) / 8: パーティクル
    auto s          = black(40);
    s.engine        = "airbrush";
    s.hardness      = 0.0f;
    s.opacity       = 0.15f;
    s.airbrush_rate = 1.5f;
    s.color         = Vec4b{20, 80, 200, 255};
    auto path       = wave(row++, VirtualPen::full);
    pen.draw(d, ink, s, std::vector<PenSample>(path.begin(), path.begin() + 80), 0.8f);
    pen.draw(d, ink, s, std::vector<PenSample>(path.begin() + 80, path.end()), 0.8f);
    auto t           = black(40);
    t.engine         = "particle";
    t.color          = Vec4b{220, 40, 120, 255};
    t.spacing        = 0.4f;
    t.particle_count = 10;
    t.seed           = 7;
    pen.draw(d, ink, t, wave(row++));
  }
  { // 9: トーン(筆圧で濃く) / 10: テクスチャ貼り(縞)
    auto s             = black(40);
    s.engine           = "tone";
    s.pressure_opacity = true;
    s.pressure_size    = false;
    s.tone_pitch       = 6;
    pen.draw(d, ink, s, VirtualPen::wave(40, W - 40, at(row++), 0, 0, 160, VirtualPen::ramp_up));
    auto t          = black(36);
    t.engine        = "pattern";
    t.pattern       = asset("pattern_stripe.png");
    t.pattern_scale = 1.0f;
    t.pressure_size = false;
    pen.draw(d, ink, t, wave(row++, VirtualPen::full));
  }
  { // 11: 水彩(赤い帯の上に青を重ねる) / 12: 消しゴム(黒い帯に隙間をあける)
    auto bar          = black(30);
    bar.color         = Vec4b{230, 60, 50, 255};
    bar.pressure_size = false;
    pen.draw(d, ink, bar, VirtualPen::line(40, at(row), 700, at(row), 60));
    auto s      = black(40);
    s.engine    = "watercolor";
    s.color     = Vec4b{30, 90, 220, 255};
    s.hardness  = 0.6f;
    s.opacity   = 0.8f;
    s.water_mix = 0.6f;
    pen.draw(d, ink, s, VirtualPen::wave(300, W - 40, at(row++), 8, 1.5f, 120, VirtualPen::bell));
    auto fat          = black(36);
    fat.pressure_size = false;
    pen.draw(d, ink, fat, VirtualPen::line(40, at(row), W - 40, at(row), 60));
    auto e     = black(30);
    e.engine   = "erase";
    e.hardness = 0.7f;
    pen.draw(d, ink, e, VirtualPen::wave(100, W - 100, at(row++), 18, 1.0f, 100, VirtualPen::full));
  }
  { // 13: プラグインのブラシ(十字) / 14: 選択範囲でクリップ(右半分だけ)
    auto s    = black(5);
    s.engine  = "cross";
    s.color   = Vec4b{0, 120, 60, 255};
    s.spacing = 1.0f;
    cutil::Prop params;
    params.set<int32_t>("arm", 5);
    s.params = params;
    pen.draw(d, ink, s, wave(row++, VirtualPen::full));
    auto sel = Selection::rect(W, H, W / 2, 0, W, H);
    d.set_clip(&sel);
    auto t  = black(34);
    t.color = Vec4b{120, 40, 160, 255};
    pen.draw(d, ink, t, wave(row++));
    d.set_clip(nullptr);
  }
  { // 15: 渦巻き(筆圧が弱→強→弱)
    auto s  = black(18);
    s.color = Vec4b{10, 10, 10, 255};
    pen.draw(d, ink, s, VirtualPen::spiral(W / 2.0f, at(row) + ROW * 0.5f, 4, 60, 4, 400, VirtualPen::bell));
  }

  CHECK(d.layers[ink].img.tile_count() > 20);
  (void)paper;

  PixelBuffer8 out;
  d.composite(out, Vec4b{248, 246, 240, 255});
  cv::Mat bgr(H, W, CV_8UC3);
  for(int y = 0; y < H; y++)
    for(int x = 0; x < W; x++) {
      const Vec4b& p          = out(x, y);
      bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(p[2], p[1], p[0]);
    }
  std::filesystem::create_directories("paint_gallery");
  CHECK(cv::imwrite("paint_gallery/gallery.png", bgr));
}
