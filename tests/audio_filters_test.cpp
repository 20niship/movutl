#include <cmath>
#include <doctest/doctest.h>
#include <movutl/app/app_impl.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/plugin/default/audio_echo_filter.hpp>
#include <movutl/plugin/default/audio_eq_filter.hpp>
#include <movutl/plugin/default/audio_reverb_filter.hpp>
#include <movutl/plugin/default/audio_volume_filter.hpp>
#include <movutl/plugin/plugin.hpp>
#include <thread>
#include <vector>

using namespace mu;

namespace {

constexpr int kN  = 200;
constexpr int kCh = 2;

// FilterPluginTableのfn_procを1回呼ぶ。stateはトラックオブジェクト毎に独立させる想定のフィルタ用に呼び出し側で保持する
bool run_proc(FilterPluginTable& plg, void** state, std::vector<int16_t>& buf, Composition* comp) {
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = comp;
  return plg.fn_proc(state, &fin, cutil::Prop());
}

std::vector<int16_t> constant_buffer(int16_t v) { return std::vector<int16_t>((size_t)kN * kCh, v); }

// 最初のサンプルだけ振幅を持つインパルス列(残りは無音)。残響/エコーの尾を観測するために使う
std::vector<int16_t> impulse_buffer() {
  std::vector<int16_t> buf((size_t)kN * kCh, 0);
  buf[0] = 30000;
  buf[1] = 30000;
  return buf;
}

} // namespace

TEST_CASE("audio_volume_filter: gainを掛けた分だけ振幅が変わる") {
  Composition comp;
  void* state = nullptr;

  auto buf = constant_buffer(1000);
  cutil::Prop p;
  p.set<float>("gain", 50.0f);
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_volume.fn_proc(&state, &fin, p));
  CHECK(buf[0] == 500);
  CHECK(buf[1] == 500);
}

TEST_CASE("audio_pan_filter: 左右に振り切ると片チャンネルが無音になる") {
  Composition comp;
  void* state = nullptr;

  auto buf = constant_buffer(1000);
  cutil::Prop p;
  p.set<float>("pan", -100.0f); // 全部左
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_pan.fn_proc(&state, &fin, p));
  CHECK(buf[0] > 900); // L: ほぼ元のまま
  CHECK(buf[1] < 100); // R: ほぼ無音

  auto buf2 = constant_buffer(1000);
  cutil::Prop p2;
  p2.set<float>("pan", 100.0f); // 全部右
  void* state2 = nullptr;
  FilterInData fin2;
  fin2.audiop   = buf2.data();
  fin2.audio_n  = kN;
  fin2.audio_ch = kCh;
  fin2.compo    = &comp;
  CHECK(mu::detail::f_audio_pan.fn_proc(&state2, &fin2, p2));
  CHECK(buf2[0] < 100);
  CHECK(buf2[1] > 900);
}

TEST_CASE("audio_eq_filter: デフォルトゲイン(全帯域100)では信号がほぼ変化しない") {
  Composition comp;
  void* state = nullptr;
  auto buf    = constant_buffer(1000);
  auto before = buf;
  CHECK(run_proc(mu::detail::f_audio_eq, &state, buf, &comp));
  for(int i = 0; i < kN * kCh; i++) CHECK(std::abs((int)buf[i] - (int)before[i]) <= 1); // low+mid+high=inを再構成するため丸め誤差程度しか出ない
}

TEST_CASE("audio_eq_filter: low_gainを0にすると信号が変化する") {
  Composition comp;
  void* state = nullptr;
  auto buf    = constant_buffer(1000);
  auto before = buf;
  cutil::Prop p;
  p.set<float>("low_gain", 0.0f);
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_eq.fn_proc(&state, &fin, p));
  bool changed = false;
  for(int i = 0; i < kN * kCh; i++)
    if(buf[i] != before[i]) changed = true;
  CHECK(changed);
}

TEST_CASE("audio_reverb_filter: mix=0では信号が変化しない") {
  Composition comp;
  void* state = nullptr;
  auto buf    = impulse_buffer();
  auto before = buf;
  cutil::Prop p;
  p.set<float>("mix", 0.0f);
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_reverb.fn_proc(&state, &fin, p));
  CHECK(buf == before);
}

TEST_CASE("audio_reverb_filter: mix>0ではインパルス後の無音区間に残響の尾が残る") {
  Composition comp;
  void* state = nullptr;
  auto buf    = impulse_buffer();
  cutil::Prop p;
  p.set<float>("mix", 80.0f);
  p.set<float>("roomsize", 80.0f);
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_reverb.fn_proc(&state, &fin, p));
  bool has_tail = false;
  for(int i = 40 * kCh; i < kN * kCh; i++) // インパルス直後を除いた区間に非ゼロが残っていれば残響が乗っている
    if(buf[i] != 0) has_tail = true;
  CHECK(has_tail);
}

TEST_CASE("audio_echo_filter: 遅延時間だけずれたところにエコーが返ってくる") {
  Composition comp; // audio_sample_rate=48000(デフォルト)
  void* state = nullptr;
  auto buf    = impulse_buffer();
  cutil::Prop p;
  p.set<float>("delay_ms", 1.0f); // 48サンプル遅延(kN=200に収まる範囲)
  p.set<float>("feedback", 50.0f);
  p.set<float>("mix", 100.0f);
  FilterInData fin;
  fin.audiop   = buf.data();
  fin.audio_n  = kN;
  fin.audio_ch = kCh;
  fin.compo    = &comp;
  CHECK(mu::detail::f_audio_echo.fn_proc(&state, &fin, p));

  int delay_samples = (int)(1.0 / 1000.0 * comp.audio_sample_rate);
  CHECK(buf[0] == 0); // mix=100なのでドライ音は消え、ディレイライン初期値(無音)がそのまま出る
  bool echoed = false;
  for(int i = delay_samples; i < delay_samples + 5 && i < kN; i++)
    if(buf[(size_t)i * kCh] != 0) echoed = true;
  CHECK(echoed);
}

// RenderWorkerPoolのOSデフォルト小スタック(macOSで512KB程度)で、旧assign(count,value)実装がSIGBUSで落ちていたのを実機確認済み。同条件で再現しないことを確認する
TEST_CASE("audio_echo_filter/audio_reverb_filter: 小さいスタックのスレッドで呼んでも落ちない") {
  Composition comp;
  bool echo_ok   = false;
  bool reverb_ok = false;

  std::thread t([&] {
    void* echo_state = nullptr;
    auto buf1        = impulse_buffer();
    cutil::Prop p1;
    p1.set<float>("delay_ms", 5.0f);
    FilterInData fin1;
    fin1.audiop   = buf1.data();
    fin1.audio_n  = kN;
    fin1.audio_ch = kCh;
    fin1.compo    = &comp;
    echo_ok       = mu::detail::f_audio_echo.fn_proc(&echo_state, &fin1, p1);

    void* reverb_state = nullptr;
    auto buf2          = impulse_buffer();
    cutil::Prop p2;
    p2.set<float>("mix", 50.0f);
    FilterInData fin2;
    fin2.audiop   = buf2.data();
    fin2.audio_n  = kN;
    fin2.audio_ch = kCh;
    fin2.compo    = &comp;
    reverb_ok     = mu::detail::f_audio_reverb.fn_proc(&reverb_state, &fin2, p2);
  });
  t.join();

  CHECK(echo_ok);
  CHECK(reverb_ok);
}

// Inspectorの手順(trk.filtersへFilterParamを積む)をそのまま再現し、mix_audio_range経由でも効果が反映されるか確認する(fn_proc直接呼び出しだけでは経路全体を検証できないため)
TEST_CASE("音声トラックにエコーフィルタをGUIと同じ手順で付けるとミキシング結果に反映される") {
  detail::register_default_filters();
  detail::activate_all_plugins();

  auto comp         = cutil::make_ref<Composition>("t", 100, 100, 30);
  auto audio        = AudioEntt::Create("a", "../assets/audio/file_example_WAV_1MG.wav");
  audio->trk.fstart = 0;
  audio->trk.fend   = 150;
  comp->insert_entity(audio);

  std::vector<int16_t> baseline((size_t)kN * kCh);
  mix_audio_range(comp.get(), 0, kN, baseline.data());

  auto* filters               = &detail::AppMain::Get()->filters;
  FilterPluginTable* echo_plg = nullptr;
  for(auto& f : *filters)
    if(std::string(f.name.c_str()) == "エコー") echo_plg = &f;
  REQUIRE(echo_plg != nullptr);

  TrackObject::FilterParam fp;
  fp.plg_ = echo_plg;
  fp.props.add_props(echo_plg->defaults);
  fp.enabled = true;
  audio->trk.filters.push_back(fp);

  std::vector<int16_t> filtered((size_t)kN * kCh);
  mix_audio_range(comp.get(), 0, kN, filtered.data());

  CHECK(baseline != filtered);
}
