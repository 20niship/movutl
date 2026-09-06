// プリコンポジション機能(#3, PR #39): CompoRefEntt/CompoAudioEnttの動作テスト
#include <doctest/doctest.h>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/compo_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/movie.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/asset/shape.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/audio_resample.hpp>
#include <movutl/render2d/renderer.hpp>

using namespace mu;

namespace {

// Composition::guidはコンストラクタで初期化されないため、テストでは必ず明示的にユニークな値を設定する
Ref<Composition> make_compo(const char* name, uint32_t guid, int w = 400, int h = 400, int fps = 30) {
  auto c  = cutil::make_ref<Composition>(name, w, h, fps);
  c->guid = guid;
  return c;
}

Ref<ShapeEntt> make_fill_rect(const char* name, Vec2 comp_size, const Vec4b& color, int fstart, int fend) {
  auto shp     = ShapeEntt::Create(name, ShapeType_Rect);
  shp->pos_    = Vec3(0, 0, 0);
  shp->size_   = comp_size;
  shp->color_  = color;
  shp->fstart_ = fstart;
  shp->fend_   = fend;
  return shp;
}

} // namespace

TEST_CASE("映像プリコンポジション: 左右にCompoRefEnttを配置し各Precompの内容が合成される") {
  Project::New();

  // Precomp1: 実動画ファイルを配置(内容は予測しないが、Main経由の結果とPrecomp1単体の結果が一致するかを見る)
  auto precomp1  = make_compo("Precomp1", 101, 400, 400, 30);
  auto movie     = Movie::Create("mov", "../assets/movies/sample-5s.mp4");
  movie->fstart_ = 0;
  movie->fend_   = 29;
  precomp1->insert_entity(movie);
  Project::Get()->compos_.push_back(precomp1);

  // Precomp2: ShapeEnttのみで構成し、厳密な色比較を可能にする
  auto precomp2 = make_compo("Precomp2", 102, 400, 400, 30);
  auto rect2    = make_fill_rect("rect2", Vec2(400, 400), Vec4b(0, 255, 0, 255), 0, 29); // BGRA順: 緑
  precomp2->insert_entity(rect2);
  Project::Get()->compos_.push_back(precomp2);

  // Main: 左半分にPrecomp1、右半分にPrecomp2を配置
  auto main = make_compo("Main", 1, 800, 400, 30);

  auto ref1              = cutil::make_ref<CompoRefEntt>();
  ref1->pos              = Vec3(0, 0, 0); // 左半分
  ref1->target_comp_guid = 101;
  ref1->fstart_          = 0;
  ref1->fend_            = 29;
  main->insert_entity(ref1);

  auto ref2              = cutil::make_ref<CompoRefEntt>();
  ref2->pos              = Vec3(400, 0, 0); // 右半分
  ref2->target_comp_guid = 102;
  ref2->fstart_          = 0;
  ref2->fend_            = 29;
  main->insert_entity(ref2);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(main.get(), 5, out));

  // Precomp1単体を同フレームでレンダリングし、Main左半分と一致するか確認する(動画の内容自体は予測しない)
  Ref<Image> precomp1_alone;
  REQUIRE(renderer.render_frame(precomp1.get(), 5, precomp1_alone));
  CHECK(out->rgba(100, 100) == precomp1_alone->rgba(100, 100));
  CHECK(out->rgba(300, 300) == precomp1_alone->rgba(300, 300));

  // Precomp2(Shapeのみ)は厳密に緑一色のはず
  CHECK(out->rgba(500, 100) == Vec4b(0, 255, 0, 255));
  CHECK(out->rgba(700, 300) == Vec4b(0, 255, 0, 255));
}

TEST_CASE("映像プリコンポジション: start_frame/speedオフセットで参照先の異なるフレームが選ばれる(speed=1.0)") {
  Project::New();

  // Sub: frame 0-9=赤, 10-19=緑, 20-29=青の四角を同じ位置に重ねる(可視フレームで切り替わる)
  auto sub = make_compo("Sub", 201, 100, 100, 30);
  sub->insert_entity(make_fill_rect("red", Vec2(100, 100), Vec4b(0, 0, 255, 255), 0, 9));     // BGRA: 赤
  sub->insert_entity(make_fill_rect("green", Vec2(100, 100), Vec4b(0, 255, 0, 255), 10, 19)); // BGRA: 緑
  sub->insert_entity(make_fill_rect("blue", Vec2(100, 100), Vec4b(255, 0, 0, 255), 20, 29));  // BGRA: 青
  Project::Get()->compos_.push_back(sub);

  auto main             = make_compo("Main", 2, 100, 100, 30);
  auto ref              = cutil::make_ref<CompoRefEntt>();
  ref->pos              = Vec3(0, 0, 0);
  ref->target_comp_guid = 201;
  ref->start_frame      = 10; // オフセット
  ref->speed            = 1.0f;
  ref->fstart_          = 0; // 計算式の基準(main_frame - fstart_)
  ref->fend_            = 29;
  main->insert_entity(ref);

  CPURenderer renderer;

  // main_frame=0 -> sub_frame = 10 + (0-0)*1 = 10 -> 緑
  Ref<Image> out0;
  REQUIRE(renderer.render_frame(main.get(), 0, out0));
  CHECK(out0->rgba(50, 50) == Vec4b(0, 255, 0, 255));

  // main_frame=5 -> sub_frame = 10 + 5 = 15 -> 緑
  Ref<Image> out5;
  REQUIRE(renderer.render_frame(main.get(), 5, out5));
  CHECK(out5->rgba(50, 50) == Vec4b(0, 255, 0, 255));

  // main_frame=10 -> sub_frame = 10 + 10 = 20 -> 青
  Ref<Image> out10;
  REQUIRE(renderer.render_frame(main.get(), 10, out10));
  CHECK(out10->rgba(50, 50) == Vec4b(255, 0, 0, 255));
}

TEST_CASE("映像プリコンポジション: start_frame/speedオフセットで参照先の異なるフレームが選ばれる(speed=0.5)") {
  Project::New();

  auto sub = make_compo("Sub", 202, 100, 100, 30);
  sub->insert_entity(make_fill_rect("red", Vec2(100, 100), Vec4b(0, 0, 255, 255), 0, 9));
  sub->insert_entity(make_fill_rect("green", Vec2(100, 100), Vec4b(0, 255, 0, 255), 10, 19));
  sub->insert_entity(make_fill_rect("blue", Vec2(100, 100), Vec4b(255, 0, 0, 255), 20, 29));
  Project::Get()->compos_.push_back(sub);

  auto main             = make_compo("Main", 3, 100, 100, 30);
  auto ref              = cutil::make_ref<CompoRefEntt>();
  ref->pos              = Vec3(0, 0, 0);
  ref->target_comp_guid = 202;
  ref->start_frame      = 10;
  ref->speed            = 0.5f;
  ref->fstart_          = 0;
  ref->fend_            = 29;
  main->insert_entity(ref);

  CPURenderer renderer;

  // main_frame=0 -> sub_frame = 10 + round(0*0.5) = 10 -> 緑
  Ref<Image> out0;
  REQUIRE(renderer.render_frame(main.get(), 0, out0));
  CHECK(out0->rgba(50, 50) == Vec4b(0, 255, 0, 255));

  // main_frame=10 -> sub_frame = 10 + round(10*0.5) = 15 -> 緑(半分の速度なのでまだ緑区間)
  Ref<Image> out10;
  REQUIRE(renderer.render_frame(main.get(), 10, out10));
  CHECK(out10->rgba(50, 50) == Vec4b(0, 255, 0, 255));

  // main_frame=20 -> sub_frame = 10 + round(20*0.5) = 20 -> 青
  Ref<Image> out20;
  REQUIRE(renderer.render_frame(main.get(), 20, out20));
  CHECK(out20->rgba(50, 50) == Vec4b(255, 0, 0, 255));
}

TEST_CASE("音声プリコンポジション: 2つのPrecompの音声を合成した結果はそれぞれ単体をミックスした結果の和になる") {
  Project::New();
  constexpr int kN  = 200;
  constexpr int kCh = 2;

  auto precomp1   = make_compo("Precomp1", 301, 100, 100, 30);
  auto audio1     = AudioEntt::Create("a1", "../assets/audio/file_example_WAV_1MG.wav");
  audio1->fstart_ = 0;
  audio1->fend_   = 150;
  precomp1->insert_entity(audio1);
  Project::Get()->compos_.push_back(precomp1);

  auto precomp2   = make_compo("Precomp2", 302, 100, 100, 30);
  auto audio2     = AudioEntt::Create("a2", "../assets/audio/file_example_WAV_1MG.wav");
  audio2->fstart_ = 0;
  audio2->fend_   = 150;
  audio2->volume_ = 50.0f; // Precomp1と区別できるよう音量を変える
  precomp2->insert_entity(audio2);
  Project::Get()->compos_.push_back(precomp2);

  auto main = make_compo("Main", 4, 100, 100, 30);

  auto cref1              = cutil::make_ref<CompoAudioEntt>();
  cref1->target_comp_guid = 301;
  cref1->fstart_          = 0;
  cref1->fend_            = 150;
  main->insert_entity(cref1);

  auto cref2              = cutil::make_ref<CompoAudioEntt>();
  cref2->target_comp_guid = 302;
  cref2->fstart_          = 0;
  cref2->fend_            = 150;
  main->insert_entity(cref2);

  std::vector<int16_t> combined((size_t)kN * kCh);
  mix_audio_range(main.get(), 0, kN, combined.data());

  std::vector<int16_t> mix1((size_t)kN * kCh);
  mix_audio_range(precomp1.get(), 0, kN, mix1.data());
  std::vector<int16_t> mix2((size_t)kN * kCh);
  mix_audio_range(precomp2.get(), 0, kN, mix2.data());

  for(int i = 0; i < kN * kCh; i++) {
    int32_t expected = std::clamp((int32_t)mix1[i] + (int32_t)mix2[i], -32768, 32767);
    CHECK(std::abs((int)combined[i] - expected) <= 2); // クリップ挙動・丸め誤差の許容
  }
}

TEST_CASE("音声プリコンポジション: start_frame/speedを適用した読み出しが直接mix_audio_range+resampleした結果と一致する") {
  Project::New();
  constexpr int kN  = 200;
  constexpr int kCh = 2;

  auto sub           = make_compo("Sub", 401, 100, 100, 30);
  auto sub_audio     = AudioEntt::Create("sa", "../assets/audio/file_example_WAV_1MG.wav");
  sub_audio->fstart_ = 0;
  sub_audio->fend_   = 300;
  sub->insert_entity(sub_audio);
  Project::Get()->compos_.push_back(sub);

  auto main              = make_compo("Main", 5, 100, 100, 30);
  auto cref              = cutil::make_ref<CompoAudioEntt>();
  cref->target_comp_guid = 401;
  cref->start_frame      = 10;   // オフセット
  cref->speed            = 2.0f; // 2倍速
  cref->fstart_          = 0;
  cref->fend_            = 300;
  main->insert_entity(cref);

  std::vector<int16_t> actual((size_t)kN * kCh);
  mix_audio_range(main.get(), 0, kN, actual.data());

  // 期待値: Subのstart_frame=10相当のサンプル位置から2倍速で読み出した結果(mix_audio_range+audio_resampleの組み合わせで公開APIのみを使って独立に再現する)
  double speed_ratio = 2.0;
  int64_t dst_start  = (int64_t)((double)0 / main->audio_sample_rate * sub->audio_sample_rate * speed_ratio) + sub->frame_to_sample(10);
  int dst_n          = (int)std::ceil((double)kN * sub->audio_sample_rate * speed_ratio / main->audio_sample_rate) + 2;
  std::vector<int16_t> dst_buf((size_t)dst_n * kCh, 0);
  mix_audio_range(sub.get(), dst_start, dst_n, dst_buf.data());

  std::vector<int16_t> expected((size_t)kN * kCh, 0);
  double rate_ratio = (double)main->audio_sample_rate / ((double)sub->audio_sample_rate * speed_ratio);
  audio_resample(dst_buf.data(), dst_n, kCh, expected.data(), kN, kCh, rate_ratio);

  for(int i = 0; i < kN * kCh; i++) CHECK(std::abs((int)actual[i] - (int)expected[i]) <= 2); // 丸め誤差の許容
}

TEST_CASE("プリコンポジション合成: ネストされたComposition内の未描画領域は透明合成され不透明背景で覆われない") {
  Project::New();

  // Sub: Composition中央に小さな矩形のみ配置し、それ以外は何も描画しない
  auto sub = make_compo("Sub", 601, 200, 200, 30);
  sub->insert_entity(make_fill_rect("small", Vec2(50, 50), Vec4b(0, 255, 0, 255), 0, 29));
  Project::Get()->compos_.push_back(sub);

  // Main: 下レイヤーに全面塗りつぶしの背景、上レイヤーにCompoRefEnttを重ねる。修正前は背景が不透明黒で覆われて見えなくなる
  auto main = make_compo("Main", 6, 200, 200, 30);
  main->insert_entity(make_fill_rect("bg", Vec2(200, 200), Vec4b(0, 0, 255, 255), 0, 29)); // BGRA: 赤
  auto ref              = cutil::make_ref<CompoRefEntt>();
  ref->pos              = Vec3(0, 0, 0);
  ref->target_comp_guid = 601;
  ref->fstart_          = 0;
  ref->fend_            = 29;
  main->insert_entity(ref);

  CPURenderer renderer;
  Ref<Image> out;
  REQUIRE(renderer.render_frame(main.get(), 0, out));

  CHECK(out->rgba(150, 150) == Vec4b(0, 0, 255, 255)); // Subの矩形が無い領域はMainの背景(赤)が透けて見える
  CHECK(out->rgba(25, 25) == Vec4b(0, 255, 0, 255));   // Subの矩形(0,0)-(50,50)の内側は緑
}

TEST_CASE("キャッシュ無効化伝播: 参照先変更後の再レンダリングで新しい参照先の内容に切り替わる") {
  Project::New();

  auto sub1 = make_compo("Sub1", 701, 100, 100, 30);
  sub1->insert_entity(make_fill_rect("red", Vec2(100, 100), Vec4b(0, 0, 255, 255), 0, 29));
  Project::Get()->compos_.push_back(sub1);

  auto sub2 = make_compo("Sub2", 702, 100, 100, 30);
  sub2->insert_entity(make_fill_rect("blue", Vec2(100, 100), Vec4b(255, 0, 0, 255), 0, 29));
  Project::Get()->compos_.push_back(sub2);

  auto main             = make_compo("Main", 7, 100, 100, 30);
  auto ref              = cutil::make_ref<CompoRefEntt>();
  ref->pos              = Vec3(0, 0, 0);
  ref->target_comp_guid = 701;
  ref->fstart_          = 0;
  ref->fend_            = 29;
  main->insert_entity(ref);

  CPURenderer renderer;
  Ref<Image> out1;
  REQUIRE(renderer.render_frame(main.get(), 0, out1));
  CHECK(out1->rgba(50, 50) == Vec4b(0, 0, 255, 255)); // Sub1=赤

  // 参照先変更時はインスペクタと同様に呼び出し元がキャッシュ無効化する責務を持つ
  ref->target_comp_guid = 702;
  main->invalidate_cache_all();

  Ref<Image> out2;
  REQUIRE(renderer.render_frame(main.get(), 0, out2));
  CHECK(out2->rgba(50, 50) == Vec4b(255, 0, 0, 255)); // Sub2=青に切り替わる
}

TEST_CASE("キャッシュ無効化伝播: 参照先Compositionの中身変更がMain側のキャッシュにも反映される") {
  Project::New();

  auto sub = make_compo("Sub", 801, 100, 100, 30);
  sub->insert_entity(make_fill_rect("red", Vec2(100, 100), Vec4b(0, 0, 255, 255), 0, 29));
  Project::Get()->compos_.push_back(sub);

  auto main             = make_compo("Main", 8, 100, 100, 30);
  auto ref              = cutil::make_ref<CompoRefEntt>();
  ref->pos              = Vec3(0, 0, 0);
  ref->target_comp_guid = 801;
  ref->fstart_          = 0;
  ref->fend_            = 29;
  main->insert_entity(ref);

  CPURenderer renderer;
  Ref<Image> out1;
  REQUIRE(renderer.render_frame(main.get(), 0, out1));
  CHECK(out1->rgba(50, 50) == Vec4b(0, 0, 255, 255)); // 変更前は赤一色

  // Sub側にEntityを追加(insert_entity内でMain側のキャッシュも伝播無効化されるはず)
  sub->insert_entity(make_fill_rect("green", Vec2(30, 30), Vec4b(0, 255, 0, 255), 0, 29));

  Ref<Image> out2;
  REQUIRE(renderer.render_frame(main.get(), 0, out2));
  CHECK(out2->rgba(15, 15) == Vec4b(0, 255, 0, 255)); // 新しく追加された緑が反映される
  CHECK(out2->rgba(50, 50) == Vec4b(0, 0, 255, 255)); // 追加範囲外は変わらず赤
}

TEST_CASE("循環参照ガード: 自己参照するCompoRefEnttがあってもクラッシュ・無限ループせず完了する") {
  Project::New();

  auto comp_a = make_compo("A", 501, 100, 100, 30);

  auto self_ref              = cutil::make_ref<CompoRefEntt>();
  self_ref->pos              = Vec3(0, 0, 0);
  self_ref->target_comp_guid = 501; // 自分自身を参照
  self_ref->fstart_          = 0;
  self_ref->fend_            = 29;
  comp_a->insert_entity(self_ref);
  Project::Get()->compos_.push_back(comp_a);

  CPURenderer renderer;
  Ref<Image> out;
  CHECK(renderer.render_frame(comp_a.get(), 0, out)); // クラッシュ/無限ループせず正常にreturnすればOK
}
