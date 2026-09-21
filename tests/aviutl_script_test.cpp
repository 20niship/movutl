#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/config.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/plugin/aviutl_script/aviutl_script_parser.hpp>
#include <movutl/plugin/plugin.hpp>

using namespace mu;
using namespace mu::detail;

TEST_CASE("aviutl_script_parser: track0/check0とスクリプト本体を分離できる") {
  std::string text = "--track0:しきい値,0,255,128,1\n"
                     "--check0:反転,1\n"
                     "@テスト効果\n"
                     "obj.putpixeldata(obj.getpixeldata())\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 1);
  CHECK(defs[0].name == "テスト効果");
  REQUIRE(defs[0].tracks.size() == 1);
  CHECK(defs[0].tracks[0].name == "しきい値");
  CHECK(defs[0].tracks[0].min_value == 0.0f);
  CHECK(defs[0].tracks[0].max_value == 255.0f);
  CHECK(defs[0].tracks[0].default_value == 128.0f);
  REQUIRE(defs[0].checks.size() == 1);
  CHECK(defs[0].checks[0].name == "反転");
  CHECK(defs[0].checks[0].default_value == true);
  CHECK(defs[0].lua_body.find("obj.putpixeldata") != std::string::npos);
  CHECK(defs[0].lua_body.find('@') == std::string::npos);
}

TEST_CASE("aviutl_script_parser: '@'ブロックが無いファイルはファイル全体を単一スクリプト(name未設定)として返す") {
  std::string text = "--track0:半径,0,500,100\n"
                     "obj.ox = obj.track0\n"
                     "obj.draw()\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 1);
  CHECK(defs[0].name.empty()); // 呼び出し側(register_aviutl_scripts)がファイル名で補完する
  REQUIRE(defs[0].tracks.size() == 1);
  CHECK(defs[0].tracks[0].name == "半径");
  CHECK(defs[0].lua_body.find("obj.draw") != std::string::npos);
}

TEST_CASE("aviutl_script_parser: 複数の@ブロックを個別に分離できる") {
  std::string text = "@効果A\n"
                     "obj.putpixeldata(obj.getpixeldata())\n"
                     "@効果B\n"
                     "obj.putpixeldata(obj.getpixeldata())\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 2);
  CHECK(defs[0].name == "効果A");
  CHECK(defs[1].name == "効果B");
}

TEST_CASE("aviutl_script_parser: --trackN/--checkNの番号が飛んでいても正しい位置に登録される") {
  std::string text = "--track2:三番目,0,10,5\n"
                     "@番号テスト\n"
                     "obj.draw()\n";

  auto defs = parse_aviutl_script(text);
  REQUIRE(defs.size() == 1);
  REQUIRE(defs[0].tracks.size() == 3);
  CHECK(defs[0].tracks[0].name.empty()); // track0/1は未定義のまま
  CHECK(defs[0].tracks[1].name.empty());
  CHECK(defs[0].tracks[2].name == "三番目");
}

namespace {
FilterPluginTable* find_filter(const char* name) {
  for(auto& f : AppMain::Get()->filters)
    if(std::string(f.name.c_str()) == name) return &f;
  return nullptr;
}

// テスト用の一時フォルダへ.anmファイルを書き出し、register_aviutl_scripts()経由(実際のフォルダ自動スキャン)で登録する
FilterPluginTable* register_test_script(const std::string& text, const char* filter_name) {
  static int counter = 0;
  auto dir           = std::filesystem::temp_directory_path() / ("movutl_aviutl_test_" + std::to_string(counter++));
  std::filesystem::create_directories(dir);
  std::ofstream ofs(dir / "test.anm");
  ofs << text;
  ofs.close();

  auto saved                     = Config::Get()->lua_script_dirs;
  Config::Get()->lua_script_dirs = {dir.string()};
  register_aviutl_scripts();
  Config::Get()->lua_script_dirs = saved;

  return find_filter(filter_name);
}

// exprsのLua式(カンマ区切り、最大16個)を評価して2x2画像のバイト列へ書き出して読み返す。スクリプトから値を取り出す手段の代わり(値は0-255の整数)
std::vector<int> probe(const std::string& exprs, FilterInData& fin, Image& img) {
  static int counter     = 0;
  std::string name       = "probe" + std::to_string(counter++);
  std::string text       = "@" + name + "\nlocal t = {" + exprs + "}\nfor i = 1, #t do t[i] = math.floor(t[i]) end\nfor i = #t + 1, 16 do t[i] = 0 end\n" + "obj.putpixeldata(string.char((table.unpack or unpack)(t)))\n";
  FilterPluginTable* plg = register_test_script(text, name.c_str());
  REQUIRE(plg != nullptr);
  img.resize(2, 2);
  fin.img = &img;
  REQUIRE(plg->fn_proc(plg, &fin, cutil::Prop{}));
  std::vector<int> out;
  for(size_t i = 0; i < 4; i++)
    for(int c = 0; c < 4; c++) out.push_back(img[i][c]);
  return out;
}
} // namespace

TEST_CASE("obj変数: w/h/screen_w/screen_h/frame/totalframe/id/index/numが実値で提供される") {
  auto comp    = cutil::make_ref<Composition>("objvar_comp", 100, 60, 30);
  auto shp     = ShapeEntt::Create("s", ShapeType_Rect);
  shp->fstart_ = 10;
  shp->fend_   = 40;
  shp->guid_   = 7;
  comp->insert_entity(shp, 0);
  comp->insert_entity(ShapeEntt::Create("s2", ShapeType_Rect), 1);
  auto shp2 = comp->layers[1].entts[0];

  Image img;
  FilterInData fin;
  fin.compo = comp.get();
  fin.entt  = shp.get();
  fin.frame = 15;
  auto v    = probe("obj.w, obj.h, obj.screen_w, obj.screen_h, obj.frame, obj.totalframe, obj.id, obj.layer, obj.index, obj.num", fin, img);
  CHECK(v[0] == 2);
  CHECK(v[1] == 2);
  CHECK(v[2] == 100);
  CHECK(v[3] == 60);
  CHECK(v[4] == 5);  // オブジェクト先頭(fstart_=10)からの相対フレーム
  CHECK(v[5] == 30); // fend_-fstart_
  CHECK(v[6] == 7);
  CHECK(v[7] == 0);
  CHECK(v[8] == 0);
  CHECK(v[9] == 1);

  fin.entt = shp2.get(); // 2レイヤー目のEntity
  auto v2  = probe("obj.layer, obj.cz, obj.aspect", fin, img);
  CHECK(v2[0] == 1);
  CHECK(v2[1] == 0);
  CHECK(v2[2] == 0);
}

TEST_CASE("obj.getinfo: AviUtl正規キーのみ対応し旧独自キーはnilを返す") {
  auto comp = cutil::make_ref<Composition>("getinfo_comp", 100, 60, 30);
  Image img;
  FilterInData fin;
  fin.compo = comp.get();
  // 多値を返すimage_maxは最後に置く(テーブルコンストラクタは末尾以外の多値を1個に切り詰める)
  auto v = probe("obj.getinfo('editing') and 1 or 0, obj.getinfo('saving') and 1 or 0, obj.getinfo('image_w') == nil and 1 or 0, obj.getinfo('clock') > 0 and 1 or 0, obj.getinfo('image_max')", fin, img);
  CHECK(v[0] == 1);
  CHECK(v[1] == 0);
  CHECK(v[2] == 1); // 旧キーimage_wは廃止
  CHECK(v[3] == 1);
  CHECK(v[4] == 100); // image_maxはCompositionサイズ(w,hの2値)
  CHECK(v[5] == 60);
}

TEST_CASE("obj.getpixel(x,y): 画素をr,g,b,aまたはcol,aで取得できる(引数なしは従来どおりw,h)") {
  Image img;
  FilterInData fin;
  // probeは2x2に作り直すため、事前に画素を仕込めない。putpixeldataで直前に書いた画素を読み戻して検証する
  auto v = probe("obj.putpixeldata(string.char(10,20,30,40, 0,0,0,0, 0,0,0,0, 0,0,0,0)) or 0, select(2, obj.getpixel()), obj.getpixel(0, 0, 'col') == 0x0A141E and 1 or 0, (select(2, obj.getpixel(0, 0, 'col'))), (select(3, obj.getpixel(0, 0))), obj.getpixel(5, 5)", fin, img);
  CHECK(v[1] == 2);  // 引数なしのgetpixel()は(w,h)
  CHECK(v[2] == 1);  // "col"は0xRRGGBB
  CHECK(v[3] == 40); // 2値目はa
  CHECK(v[4] == 30); // 3値目はb
  CHECK(v[5] == 0);  // 範囲外は0
}

TEST_CASE("obj.putpixel: 指定画素だけを書き換え、範囲外は無視する") {
  std::string text       = "@putpixelテスト\n"
                           "obj.putpixel(1, 0, 255, 128, 64)\n"
                           "obj.putpixel(0, 1, 1, 2, 3, 4)\n"
                           "obj.putpixel(9, 9, 255, 255, 255)\n";
  FilterPluginTable* plg = register_test_script(text, "putpixelテスト");
  REQUIRE(plg != nullptr);
  Image img(2, 2);
  img.fill_rgba(Vec4b(0, 0, 0, 0));
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));
  CHECK(img(1, 0) == Vec4b(255, 128, 64, 255)); // aの既定は255
  CHECK(img(0, 1) == Vec4b(1, 2, 3, 4));
  CHECK(img(0, 0) == Vec4b(0, 0, 0, 0));
}

TEST_CASE("obj.copypixel: 画素を別位置へコピーし範囲外は無視する") {
  std::string text       = "@copypixelテスト\n"
                           "obj.copypixel(0, 0, 1, 1)\n"
                           "obj.copypixel(1, 0, 5, 5)\n";
  FilterPluginTable* plg = register_test_script(text, "copypixelテスト");
  REQUIRE(plg != nullptr);
  Image img(2, 2);
  img.fill_rgba(Vec4b(0, 0, 0, 255));
  img(1, 1) = Vec4b(9, 8, 7, 6);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));
  CHECK(img(0, 0) == Vec4b(9, 8, 7, 6));
  CHECK(img(1, 0) == Vec4b(0, 0, 0, 255)); // 範囲外のコピー元は無視
}

TEST_CASE("obj.rand: 範囲内の整数を返し、seed指定時は同じ(seed,frame)で同じ値・省略時は呼び出し毎に変わるが再実行で再現する") {
  Image img;
  FilterInData fin;
  fin.frame = 3;
  auto v1   = probe("obj.rand(10, 20, 1, 0), obj.rand(10, 20, 1, 0), obj.rand(10, 20, 2, 0), obj.rand(5, 5), obj.rand(0, 100), obj.rand(0, 100)", fin, img);
  auto v2   = probe("obj.rand(10, 20, 1, 0), obj.rand(10, 20, 1, 0), obj.rand(10, 20, 2, 0), obj.rand(5, 5), obj.rand(0, 100), obj.rand(0, 100)", fin, img);
  for(int i = 0; i < 3; i++) {
    CHECK(v1[i] >= 10);
    CHECK(v1[i] <= 20);
  }
  CHECK(v1[0] == v1[1]); // 同じseed/frameは同じ値
  CHECK(v1[3] == 5);     // min==max
  CHECK(v1 == v2);       // 同じフレームで再実行すると同じ結果(ctx毎に連番が0から)
  bool differs = false;  // seed省略の連続呼び出し・別seed・別フレームで値が全て同じになることはない(決定的な固定入力なので偶然の一致で落ちることもない)
  differs      = differs || v1[4] != v1[5] || v1[0] != v1[2];
  CHECK(differs);
}

TEST_CASE("obj.interpolation/getvalue: スプライン補間と現在値取得") {
  Image img;
  FilterInData fin;
  auto v = probe("obj.getvalue('zoom') * 10, obj.getvalue('nothing') == nil and 1 or 0, obj.interpolation(0, 0,0,0, 10,20,30, 20,40,60, 30,60,90), obj.interpolation(1, 0,0,0, 10,20,30, 20,40,60, 30,60,90)", fin, img);
  CHECK(v[0] == 10); // zoom既定1
  CHECK(v[1] == 1);
  CHECK(v[2] == 10); // t=0でp1
}

TEST_CASE("register_aviutl_scripts: 2値化スクリプトをフォルダスキャン経由でフィルタとして登録・実行できる") {
  std::string text = "--track0:しきい値,0,255,128,1\n"
                     "@AviUtlテスト2値化\n"
                     "local unpack = table.unpack or unpack\n"
                     "local w = obj.w\n"
                     "local h = obj.h\n"
                     "local px = obj.getpixeldata()\n"
                     "local buf = {}\n"
                     "for i = 1, w*h do\n"
                     "  local o = (i-1)*4\n"
                     "  local r,g,b,a = string.byte(px, o+1, o+4)\n"
                     "  local v = (r+g+b)/3 >= track0 and 255 or 0\n"
                     "  buf[o+1], buf[o+2], buf[o+3], buf[o+4] = v, v, v, a\n"
                     "end\n"
                     "obj.putpixeldata(string.char(unpack(buf)))\n";

  FilterPluginTable* plg = register_test_script(text, "AviUtlテスト2値化");
  REQUIRE(plg != nullptr);
  REQUIRE(plg->fn_proc != nullptr);
  REQUIRE(plg->props.fields.size() == 1);
  CHECK(std::string(plg->props.fields[0].name) == "しきい値");
  CHECK(plg->guid != 0); // 安定したguidが振られていること

  Image img(2, 2);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255); // 平均200

  FilterInData fin;
  fin.img = &img;

  cutil::Prop p = plg->defaults;
  p.set<float>("しきい値", 128.0f);
  CHECK(plg->fn_proc(plg, &fin, p));
  CHECK(img[0][0] == 255); // 200 >= 128 なので白

  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 200, 200, 255); // 1回目の実行結果で上書きされているので再初期化
  p.set<float>("しきい値", 250.0f);
  CHECK(plg->fn_proc(plg, &fin, p));
  CHECK(img[0][0] == 0); // 200 < 250 なので黒
}

TEST_CASE("register_aviutl_scripts: obj.drawpolyで台形変形を実行できる") {
  std::string text = "@あおりテスト\n"
                     "local w, h = obj.w, obj.h\n"
                     "obj.drawpoly(-2, -h / 2, 0, 2, -h / 2, 0, w / 2, h / 2, 0, -w / 2, h / 2, 0)\n";

  FilterPluginTable* plg = register_test_script(text, "あおりテスト");
  REQUIRE(plg != nullptr);

  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(255, 255, 255, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img(5, 0)[3] > 0);  // 狭めた上辺の中央付近は残る
  CHECK(img(0, 0)[3] == 0); // 上辺左端は狭められた範囲外で透明になる
}

TEST_CASE("register_aviutl_scripts: obj.drawで移動・拡大縮小を実行できる") {
  std::string text = "@移動テスト\n"
                     "obj.ox = 3\n"
                     "obj.zoom = 1\n"
                     "obj.draw()\n";

  FilterPluginTable* plg = register_test_script(text, "移動テスト");
  REQUIRE(plg != nullptr);

  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 0);
  img(2, 5) = Vec4b(255, 0, 0, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img(5, 5)[0] == 255); // ox=3だけ右へ移動しているはず(2+3=5)
  CHECK(img(2, 5)[3] == 0);   // 元の位置は透明にクリアされている
}

TEST_CASE("register_aviutl_scripts: obj.draw()を明示的に呼ばなくてもobj.ox等の設定だけで暗黙的に移動が反映される(AviUtl本体の実仕様)") {
  std::string text = "@暗黙drawテスト\n"
                     "obj.ox = 3\n"; // obj.draw()を一切呼ばない典型的なAviUtlスクリプトパターン(例: 実際に配布されているAutoCircle.anm)

  FilterPluginTable* plg = register_test_script(text, "暗黙drawテスト");
  REQUIRE(plg != nullptr);

  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 0);
  img(2, 5) = Vec4b(255, 0, 0, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img(5, 5)[0] == 255); // obj.draw()の呼び出しが無くてもox=3の移動が反映される
}

TEST_CASE("register_aviutl_scripts: obj.aspect(縦横比)とobj.ryが暗黙drawに反映される") {
  auto count_row = [](const Image& img, int y) {
    int n = 0;
    for(size_t x = 0; x < img.width; x++) n += img(x, y)[3] > 0;
    return n;
  };
  auto run = [&](const char* script, const char* name) {
    FilterPluginTable* plg = register_test_script(script, name);
    REQUIRE(plg != nullptr);
    Image img(20, 20);
    for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(255, 0, 0, 255);
    FilterInData fin;
    fin.img = &img;
    REQUIRE(plg->fn_proc(plg, &fin, cutil::Prop{}));
    return count_row(img, 10);
  };
  CHECK(run("@縦横比0\nobj.oy = 0\n", "縦横比0") == 20);
  CHECK(run("@縦横比テスト\nobj.aspect = 0.5\n", "縦横比テスト") == 10); // 正で横が縮む
  CHECK(run("@Y軸回転テスト\nobj.ry = 60\n", "Y軸回転テスト") < 20);
}

TEST_CASE("register_aviutl_scripts: obj.cx(基点)は画像中心からのオフセットとして暗黙drawに反映される") {
  FilterPluginTable* plg = register_test_script("@基点テスト\nobj.cx = 2\n", "基点テスト");
  REQUIRE(plg != nullptr);

  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 0);
  img(7, 5) = Vec4b(255, 0, 0, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img(5, 5)[0] == 255); // 基点が右へ2pxずれる分、基点をox=0に置くと画像は左へ2px動く
}

TEST_CASE("Entity::composite: 基点まわりに回転する(基点が画像中心なら位置は動かない、ずらすと画像中心が動く)") {
  Image src(10, 10);
  src.fill_rgba(Vec4b(255, 0, 0, 255));
  auto mk = [] {
    auto t = cutil::make_ref<Image>(100, 100);
    t->fill_rgba(Vec4b(0, 0, 0, 0));
    return t;
  };
  auto ent       = cutil::make_ref<Image>();
  ent->rotation_ = 90.f;
  auto a         = mk();
  REQUIRE(ent->composite(src, a.get()));
  CHECK(a->rgba(50, 50)[0] == 255); // 基点=中心: 中央に残る

  ent->anchor_ = Vec3(20, 0, 0); // 基点を右へ20px。基点は中心(50,50)に固定され、90度回転で画像中心は基点の上(50,30)へ回る
  auto b       = mk();
  REQUIRE(ent->composite(src, b.get()));
  CHECK(b->rgba(50, 50)[0] == 0);
  CHECK(b->rgba(50, 30)[0] == 255);
}

TEST_CASE("register_aviutl_scripts: obj.copybufferで画像バッファを退避・復元できる") {
  std::string text = "@バッファテスト\n"
                     "obj.copybuffer(\"cache:saved\", \"obj\")\n"
                     "local px = obj.getpixeldata()\n"
                     "obj.putpixeldata(string.rep(string.char(0,0,0,0), #px / 4))\n" // いったん全消去
                     "obj.copybuffer(\"obj\", \"cache:saved\")\n";                   // 退避しておいた内容を復元

  FilterPluginTable* plg = register_test_script(text, "バッファテスト");
  REQUIRE(plg != nullptr);

  Image img(2, 2);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 100, 50, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img[0] == Vec4b(200, 100, 50, 255)); // 退避->復元で元の内容が戻る
}

TEST_CASE("register_aviutl_scripts: obj.copybuffer(obj,tmp)がフレーム開始時点のバッファを暗黙的に復元する") {
  std::string text = "@tmp復元テスト\n"
                     "obj.putpixeldata(string.rep(string.char(0,0,0,0), 4))\n" // まず全消去
                     "obj.copybuffer(\"obj\", \"tmp\")\n";                     // フレーム開始時点(元の内容)へ戻す

  FilterPluginTable* plg = register_test_script(text, "tmp復元テスト");
  REQUIRE(plg != nullptr);

  Image img(2, 2);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(200, 100, 50, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img[0] == Vec4b(200, 100, 50, 255)); // "tmp"は明示的にcopybufferしなくてもフレーム開始時点の内容を持つ
}

TEST_CASE("register_aviutl_scripts: obj.setoption(drawtarget,tempbuffer)でバッファを拡張できる") {
  std::string text = "@拡張テスト\n"
                     "obj.setoption(\"drawtarget\", \"tempbuffer\", 20, 20)\n"
                     "local w, h = obj.getpixel()\n"
                     "obj.putpixeldata(string.rep(string.char(255,255,255,255), w * h))\n";

  FilterPluginTable* plg = register_test_script(text, "拡張テスト");
  REQUIRE(plg != nullptr);

  Image img(10, 10);
  for(size_t i = 0; i < img.size(); i++) img[i] = Vec4b(0, 0, 0, 255);
  FilterInData fin;
  fin.img = &img;
  CHECK(plg->fn_proc(plg, &fin, cutil::Prop{}));

  CHECK(img.width == 20);
  CHECK(img.height == 20);
}
