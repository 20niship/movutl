#include <algorithm>
#include <maximilian.h>
#include <movutl/asset/composition.hpp>
#include <movutl/plugin/default/audio_echo_filter.hpp>
#include <vector>

#define GUID(x) (0x0000000100000000 | x)

namespace mu::detail {

namespace {

struct EchoState {
  std::vector<maxiDelayline> lines; // チャンネルごとに独立したディレイライン
};

bool echo_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_ASSERT(fpip != nullptr && fpip->audiop != nullptr);
  auto* slot = (void**)fp;
  if(*slot == nullptr) *slot = new EchoState();
  auto* st = (EchoState*)*slot;
  if((int)st->lines.size() != fpip->audio_ch) st->lines.assign(std::max(1, fpip->audio_ch), maxiDelayline());

  float delay_ms = std::max(1.0f, cutil::get_or<float>(p, "delay_ms", 300.0f));
  float feedback = std::clamp(cutil::get_or<float>(p, "feedback", 40.0f) / 100.0f, 0.0f, 0.95f);
  float mix      = std::clamp(cutil::get_or<float>(p, "mix", 30.0f) / 100.0f, 0.0f, 1.0f);
  int sr           = fpip->compo ? fpip->compo->audio_sample_rate : 48000;
  int delay_samples = std::max(1, (int)(delay_ms / 1000.0f * sr));

  for(int i = 0; i < fpip->audio_n; i++) {
    for(int c = 0; c < fpip->audio_ch; c++) {
      int idx        = i * fpip->audio_ch + c;
      double in      = fpip->audiop[idx] / 32768.0;
      double wet     = st->lines[c].dl(in, delay_samples, feedback);
      double out     = in * (1.0 - mix) + wet * mix;
      fpip->audiop[idx] = (int16_t)std::clamp((int32_t)(out * 32768.0), -32768, 32767);
    }
  }
  return true;
}

bool echo_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  auto add = [&](const char* name, const char* label, float mn, float mx, float def) {
    props->fields.push_back(cutil::PropInfo::Field(name, 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(label);
    props->fields.back().min_value = mn;
    props->fields.back().max_value = mx;
    defaults->set<float>(name, def);
  };
  add("delay_ms", "ディレイ時間(ms)", 1.0f, 2000.0f, 300.0f);
  add("feedback", "フィードバック", 0.0f, 95.0f, 40.0f);
  add("mix", "Wet/Dryミックス", 0.0f, 100.0f, 30.0f);
  return true;
}

} // namespace

FilterPluginTable f_audio_echo = {
  GUID(0x0005),
  FilterAudioOnly,
  cutil::Str("エコー"),
  cutil::Str("ディレイ+フィードバックによるエコーエフェクト(Maximilian maxiDelayline使用)"),
  0, "0",
  nullptr, nullptr,
  echo_init, nullptr,
  echo_proc, nullptr,
  nullptr, nullptr, nullptr,
};

} // namespace mu::detail
