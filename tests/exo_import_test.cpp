#include <doctest/doctest.h>
#include <filesystem>
#include <fstream>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/image.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/asset/text.hpp>
#include <movutl/command/exo/exo_import.hpp>
#include <movutl/core/command.hpp>
#include <set>

using namespace mu;

namespace {
// import_exoコマンドを実行し、追加されたEntity数を返す(コマンド失敗時は-1)
int import_exo_file(const char* path) {
  register_exo_command();
  auto count = []() {
    int n = 0;
    for(auto& l : Composition::GetActiveComp()->layers) n += (int)l.entts.size();
    return n;
  };
  int before = count();
  if(!run_command("import_exo", path)) return -1;
  return count() - before;
}
} // namespace

TEST_CASE("exo: import_exo_file") {
  Project::New();
  auto* comp = Composition::GetActiveComp();
  REQUIRE(comp != nullptr);
  // 音声ファイル(存在しないWindowsパス) + テキスト("テスト"はCP932ではなくtext=のUTF-16LE hex)
  std::string exo = "[exedit]\r\nwidth=640\r\nheight=360\r\nrate=30\r\nscale=1\r\nlength=100\r\n"
                    "[0]\r\nstart=1\r\nend=10\r\nlayer=1\r\naudio=1\r\n"
                    "[0.0]\r\n_name=\x89\xb9\x90\xba\x83\x74\x83\x40\x83\x43\x83\x8b\r\n\x8d\xc4\x90\xb6\x88\xca\x92\x75=1.5\r\nfile=C:\\no\\such\\a.mp3\r\n"
                    "[0.1]\r\n_name=\x95\x57\x8f\x80\x8d\xc4\x90\xb6\r\n\x89\xb9\x97\xca=80.0,80.0,1\r\n"
                    "[1]\r\nstart=5\r\nend=20\r\nlayer=3\r\n"
                    "[1.0]\r\n_name=\x83\x65\x83\x4c\x83\x58\x83\x67\r\ntext=c630b930c830000000000000\r\ncolor=ff0000\r\nfont=MS UI Gothic\r\n"
                    "[1.1]\r\n_name=\x95\x57\x8f\x80\x95\x60\x89\xe6\r\nX=10.0\r\nY=20.0\r\nZ=0.0\r\n\x8Ag\x91\xe5\x97\xa6=200.00\r\n\x93\xa7\x96\xbe\x93x=50.0\r\n";
  auto path       = std::filesystem::temp_directory_path() / "movutl_test.exo";
  {
    std::ofstream ofs(path, std::ios::binary);
    ofs << exo;
  }
  CHECK(import_exo_file(path.string().c_str()) == 2);
  REQUIRE(comp->layers.size() >= 3);

  auto* audio = dynamic_cast<AudioEntt*>(comp->layers[0].entts.at(0).get());
  REQUIRE(audio != nullptr);
  CHECK(audio->fstart_ == 0);
  CHECK(audio->fend_ == 9);
  CHECK(audio->offset_sec_ == doctest::Approx(1.5));
  CHECK(audio->volume_ == doctest::Approx(80.f));

  CHECK(comp->layers[1].entts.empty());
  auto* text = dynamic_cast<TextEntt*>(comp->layers[2].entts.at(0).get());
  REQUIRE(text != nullptr);
  CHECK(text->text == "テスト");
  CHECK(text->fstart_ == 4);
  CHECK(text->fend_ == 19);
  CHECK(text->scale_[0] == doctest::Approx(200.f));
  CHECK(text->alpha_ == doctest::Approx(0.5f).epsilon(0.01));
  CHECK(text->guid_ != 0);

  CHECK(import_exo_file("/nonexistent/x.exo") == -1);
  std::filesystem::remove(path);
}

namespace {
namespace fs = std::filesystem;

// tests/data/exo/comprehensive.exo と実素材(assets/)を一時ディレクトリへ並べる。
// 相対パス/Windows絶対パス(階層を削って探索)/存在しないパスの解決を確認するための配置
fs::path setup_exo_dir() {
  auto dir = fs::temp_directory_path() / "movutl_exo_comprehensive";
  fs::remove_all(dir);
  fs::create_directories(dir / "media" / "sub");
  auto cp = [&](const char* from, const fs::path& to) { fs::copy_file(from, dir / to, fs::copy_options::overwrite_existing); };
  cp("../tests/data/exo/comprehensive.exo", "comprehensive.exo");
  cp("../assets/movies/big_buck_bunny_360_10s.mp4", "media/movie1.mp4");
  cp("../assets/movies/sample-5s.mp4", "media/movie2.mp4");
  cp("../assets/images/blender_png.png", "media/image1.png");
  cp("../assets/images/blender_png.png", "image2.png"); // D:\素材\画像\image2.png はexoと同じ階層で見つかる
  cp("../assets/images/blender_png.png", "media/sub/image3.png");
  cp("../assets/audio/file_example_WAV_1MG.wav", "media/audio1.wav");
  cp("../assets/audio/file_example_WAV_1MG.wav", "audio1.wav"); // C:\Users\someone\Music\audio1.wav
  return dir;
}

std::vector<Ref<Entity>> layer_entts(Composition* c, int layer) { return layer < (int)c->layers.size() ? c->layers[layer].entts : std::vector<Ref<Entity>>{}; }
} // namespace

TEST_CASE("exo: comprehensive.exo (動画/画像/音声/テキスト/図形を複数レイヤーへ)") {
  auto dir = setup_exo_dir();
  Project::New();
  auto* comp = Composition::GetActiveComp();
  REQUIRE(comp != nullptr);
  // 20オブジェクト中、カメラ制御(未対応)1個を除く19個
  CHECK(import_exo_file((dir / "comprehensive.exo").string().c_str()) == 19);

  SUBCASE("Compositionの範囲が全Entityのmin/maxになる") {
    CHECK(comp->fstart == 0); // 最小のstart=1(0始まり換算で0)
    CHECK(comp->fend == 149); // 最大のend=150
  }

  SUBCASE("動画: 相対/Windows絶対/../含む相対パスが解決される") {
    auto l1 = layer_entts(comp, 0);
    REQUIRE(l1.size() == 2);
    auto* m1 = dynamic_cast<Movie*>(l1[0].get());
    auto* m2 = dynamic_cast<Movie*>(l1[1].get());
    REQUIRE((m1 && m2));
    CHECK(fs::exists(m1->path_));
    CHECK(fs::exists(m2->path_)); // C:\Users\someone\Desktop\project\media\movie2.mp4 -> <exo>/media/movie2.mp4
    CHECK(m1->get_input_plugin() != nullptr);
    CHECK(m2->get_input_plugin() != nullptr);
    CHECK(m1->fstart_ == 0);
    CHECK(m1->fend_ == 59);
    CHECK(m1->pos_[0] == doctest::Approx(10.f));
    CHECK(m1->scale_[0] == doctest::Approx(120.f));
    CHECK(m1->alpha_ == doctest::Approx(0.9f)); // 透明度10%
    CHECK(m2->speed == doctest::Approx(50.f));
    CHECK(m2->loop_);
    CHECK(m2->start_frame_ == 31);

    auto l2 = layer_entts(comp, 1);
    REQUIRE(l2.size() == 1);
    auto* m3 = dynamic_cast<Movie*>(l2[0].get());
    REQUIRE(m3 != nullptr);
    CHECK(fs::exists(m3->path_));
    CHECK(m3->get_input_plugin() != nullptr);
  }

  SUBCASE("画像") {
    auto l3 = layer_entts(comp, 2);
    REQUIRE(l3.size() == 2);
    for(auto& e : l3) {
      auto* img = dynamic_cast<Image*>(e.get());
      REQUIRE(img != nullptr);
      CHECK(fs::exists(img->path));
      CHECK(img->width > 0);
    }
    auto* i2 = dynamic_cast<Image*>(l3[1].get());
    CHECK(i2->alpha_ == doctest::Approx(0.75f).epsilon(0.01));
    CHECK(i2->scale_[0] == doctest::Approx(50.f));
    auto l4 = layer_entts(comp, 3);
    REQUIRE(l4.size() == 1);
    CHECK(dynamic_cast<Image*>(l4[0].get())->width > 0); // media/sub/image3.png
  }

  SUBCASE("音声") {
    auto a1 = dynamic_cast<AudioEntt*>(layer_entts(comp, 4).at(0).get());
    auto a2 = dynamic_cast<AudioEntt*>(layer_entts(comp, 5).at(0).get());
    REQUIRE((a1 && a2));
    CHECK(fs::exists(a1->path_));
    CHECK(fs::exists(a2->path_));
    CHECK(a1->volume_ == doctest::Approx(100.f));
    CHECK(a2->volume_ == doctest::Approx(75.f));
    CHECK(a2->offset_sec_ == doctest::Approx(2.5));
    CHECK(a2->speed == doctest::Approx(200.f));
    CHECK(a2->loop_);
  }

  SUBCASE("テキスト: 日本語/サロゲートペア/改行/色") {
    auto l7 = layer_entts(comp, 6);
    REQUIRE(l7.size() == 2);
    auto* t1 = dynamic_cast<TextEntt*>(l7[0].get());
    auto* t2 = dynamic_cast<TextEntt*>(l7[1].get());
    REQUIRE((t1 && t2));
    CHECK(t1->text == "こんにちは世界");
    CHECK(t1->color_[0] == 255);
    CHECK(t1->color_[1] == 0);
    CHECK(t1->pos_[0] == doctest::Approx(-199.f));
    CHECK(t1->pos_[1] == doctest::Approx(118.f));
    CHECK(t2->text == "Hello, exo! \xF0\x9F\x98\x80\xF0\xA0\xAE\xB7");
    CHECK(t2->color_[1] == 255);
    auto* t3 = dynamic_cast<TextEntt*>(layer_entts(comp, 7).at(0).get());
    REQUIRE(t3 != nullptr);
    CHECK(t3->text == "ＭＵＬＴＩ\r\nLINE");
    CHECK(t3->scale_[0] == doctest::Approx(150.f));
    CHECK(dynamic_cast<TextEntt*>(layer_entts(comp, 8).at(0).get())->text == "背景の字幕テキスト");
  }

  SUBCASE("図形: 種類マッピング") {
    const ShapeType expect[] = {ShapeType_Rect, ShapeType_Circle, ShapeType_Rect, ShapeType_Triangle, ShapeType_Hexagon, ShapeType_Hexagon};
    for(int i = 0; i < 6; i++) {
      auto* s = dynamic_cast<ShapeEntt*>(layer_entts(comp, 9 + i).at(0).get());
      REQUIRE(s != nullptr);
      CHECK(s->shape_type_ == expect[i]);
      CHECK(s->size_[0] == doctest::Approx(50.f + i * 20));
      CHECK(s->fstart_ == i * 10);
      CHECK(s->guid_ != 0);
    }
  }

  SUBCASE("素材が見つからなくてもトラックは作られる/カメラ制御はスキップ") {
    CHECK(layer_entts(comp, 15).empty()); // layer=16 カメラ制御
    auto l = layer_entts(comp, 16);
    REQUIRE(l.size() == 1);
    auto* m = dynamic_cast<Movie*>(l[0].get());
    REQUIRE(m != nullptr);
    CHECK(m->get_input_plugin() == nullptr);
  }

  SUBCASE("全Entityにguidが振られ重複しない") {
    std::set<uint64_t> guids;
    for(auto& l : comp->layers)
      for(auto& e : l.entts) CHECK(guids.insert(e->guid_).second);
    CHECK(guids.size() == 19);
  }

  fs::remove_all(dir);
}

TEST_CASE("exo: 既存Entityと重ならない位置までレイヤーを下げて配置する") {
  auto dir = setup_exo_dir();
  auto exo = (dir / "comprehensive.exo").string();
  Project::New();
  auto* comp = Composition::GetActiveComp();
  REQUIRE(comp != nullptr);
  // 既存トラック: layer0にフレーム0-500の長いテキスト
  auto t     = TextEntt::Create("existing");
  t->fstart_ = 0;
  t->fend_   = 500;
  comp->insert_entity(t, 0);

  REQUIRE(import_exo_file(exo.c_str()) == 19);
  CHECK(comp->layers[0].entts.size() == 1); // 既存layer0にはexoのものが入らない
  CHECK(comp->layers[0].entts[0].get() == t.get());
  CHECK(comp->layers[1].entts.size() == 2); // exoのlayer1(動画2つ)が1つ下がる

  // もう一度取り込んでも、どのレイヤーでもフレーム範囲が重ならない
  REQUIRE(import_exo_file(exo.c_str()) == 19);
  for(auto& l : comp->layers)
    for(size_t i = 0; i < l.entts.size(); i++)
      for(size_t j = i + 1; j < l.entts.size(); j++) CHECK((l.entts[i]->fend_ < l.entts[j]->fstart_ || l.entts[j]->fend_ < l.entts[i]->fstart_));
  CHECK(comp->fend == 500);
  fs::remove_all(dir);
}
