#include <algorithm>
#include <libs/maxiReverb.h>
#include <maximilian.h>
#include <movutl/plugin/default/audio_filters.hpp>
#include <vector>

#define GUID(x) (0x0000000100000000 | x)

namespace mu::detail {

namespace {

struct ReverbState {
  std::vector<maxiFreeVerb> verbs; // チャンネルごとに独立したインスタンス
};

bool reverb_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_ASSERT(fpip != nullptr && fpip->audiop != nullptr);
  auto* slot = (void**)fp;
  if(*slot == nullptr) *slot = new ReverbState();
  auto* st = (ReverbState*)*slot;
  if((int)st->verbs.size() != fpip->audio_ch) st->verbs.resize(std::max(1, fpip->audio_ch)); // assign(count,value)によるスタック上の一時構築を避ける

  float roomsize = std::clamp(cutil::get_or<float>(p, "roomsize", 50.0f) / 100.0f, 0.0f, 1.0f);
  float absorb   = std::clamp(cutil::get_or<float>(p, "absorbtion", 50.0f) / 100.0f, 0.0f, 1.0f);
  float mix      = std::clamp(cutil::get_or<float>(p, "mix", 30.0f) / 100.0f, 0.0f, 1.0f);

  for(int i = 0; i < fpip->audio_n; i++) {
    for(int c = 0; c < fpip->audio_ch; c++) {
      int idx           = i * fpip->audio_ch + c;
      double in         = fpip->audiop[idx] / 32768.0;
      double wet        = st->verbs[c].play(in, roomsize, absorb);
      double out        = in * (1.0 - mix) + wet * mix;
      fpip->audiop[idx] = (int16_t)std::clamp((int32_t)(out * 32768.0), -32768, 32767);
    }
  }
  return true;
}

bool reverb_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  auto add = [&](const char* name, const char* label, float mn, float mx, float def) {
    props->fields.push_back(cutil::PropInfo::Field(name, 0, cutil::prop_info_of<float>()));
    props->fields.back().set_label(label);
    props->fields.back().min_value = mn;
    props->fields.back().max_value = mx;
    defaults->set<float>(name, def);
  };
  add("roomsize", "部屋の広さ", 0.0f, 100.0f, 50.0f);
  add("absorbtion", "吸収率", 0.0f, 100.0f, 50.0f);
  add("mix", "Wet/Dryミックス", 0.0f, 100.0f, 30.0f);
  return true;
}

} // namespace

FilterPluginTable f_audio_reverb = {
  GUID(0x0004), FilterAudioOnly, cutil::Str("Reverb"), cutil::Str("残響エフェクト(Maximilian maxiFreeVerb使用)"), 0, "0", nullptr, nullptr, reverb_init, nullptr, reverb_proc, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
