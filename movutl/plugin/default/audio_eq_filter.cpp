#include <algorithm>
#include <maximilian.h>
#include <movutl/plugin/default/audio_eq_filter.hpp>

#define GUID(x) (0x0000000100000000 | x)

namespace mu::detail {

namespace {

// 3バンドEQ: 低域(lores)と高域(hires)を抽出し、残りを中域として3バンド分の利得を掛けて合成する
struct EqState {
  maxiFilter lo;
  maxiFilter hi;
};

bool eq_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_ASSERT(fpip != nullptr && fpip->audiop != nullptr);
  auto* slot = (void**)fp;
  if(*slot == nullptr) *slot = new EqState();
  auto* st = (EqState*)*slot;

  float low_freq  = cutil::get_or<float>(p, "low_freq", 300.0f);
  float high_freq = cutil::get_or<float>(p, "high_freq", 3000.0f);
  float low_gain  = cutil::get_or<float>(p, "low_gain", 100.0f) / 100.0f;
  float mid_gain  = cutil::get_or<float>(p, "mid_gain", 100.0f) / 100.0f;
  float high_gain = cutil::get_or<float>(p, "high_gain", 100.0f) / 100.0f;

  int total = fpip->audio_n * fpip->audio_ch;
  for(int i = 0; i < total; i++) {
    double in       = fpip->audiop[i] / 32768.0;
    double lowpart  = st->lo.lores(in, low_freq, 0.5);
    double highpart = st->hi.hires(in, high_freq, 0.5);
    double midpart  = in - lowpart - highpart;
    double out      = lowpart * low_gain + midpart * mid_gain + highpart * high_gain;
    fpip->audiop[i] = (int16_t)std::clamp((int32_t)(out * 32768.0), -32768, 32767);
  }
  return true;
}

bool eq_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  auto add = [&](const char* name, const char* label, float mn, float mx, float def) {
    props->fields.push_back(cutil::PropInfo::Field(name, 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(label);
    props->fields.back().min_value = mn;
    props->fields.back().max_value = mx;
    defaults->set<float>(name, def);
  };
  add("low_freq", "低域クロスオーバー(Hz)", 20.0f, 2000.0f, 300.0f);
  add("high_freq", "高域クロスオーバー(Hz)", 1000.0f, 20000.0f, 3000.0f);
  add("low_gain", "低域ゲイン", 0.0f, 400.0f, 100.0f);
  add("mid_gain", "中域ゲイン", 0.0f, 400.0f, 100.0f);
  add("high_gain", "高域ゲイン", 0.0f, 400.0f, 100.0f);
  return true;
}

} // namespace

FilterPluginTable f_audio_eq = {
  GUID(0x0003), FilterAudioOnly, cutil::Str("EQ"), cutil::Str("3バンドイコライザー(Maximilian maxiFilter使用)"), 0, "0", nullptr, nullptr, eq_init, nullptr, eq_proc, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
