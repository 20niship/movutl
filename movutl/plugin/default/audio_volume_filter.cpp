#include <algorithm>
#include <cmath>
#include <movutl/plugin/default/audio_volume_filter.hpp>

#define GUID(x) (0x0000000100000000 | x)

namespace mu::detail {

namespace {

bool volume_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr && fpip->audiop != nullptr);
  float gain = cutil::get_or<float>(p, "gain", 100.0f) / 100.0f;
  int total  = fpip->audio_n * fpip->audio_ch;
  for(int i = 0; i < total; i++) fpip->audiop[i] = (int16_t)std::clamp((int32_t)(fpip->audiop[i] * gain), -32768, 32767);
  return true;
}

bool volume_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  props->fields.push_back(cutil::PropInfo::Field("gain", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("音量");
  props->fields.back().min_value  = 0.0f;
  props->fields.back().max_value  = 400.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("gain", 100.0f);
  return true;
}

bool pan_proc(void* fp, FilterInData* fpip, const cutil::Prop& p) {
  MU_UNUSED(fp);
  MU_ASSERT(fpip != nullptr && fpip->audiop != nullptr);
  if(fpip->audio_ch != 2) return true; // ステレオ以外は無変換
  float pan   = std::clamp(cutil::get_or<float>(p, "pan", 0.0f) / 100.0f, -1.0f, 1.0f);
  float angle = (pan + 1.0f) * 0.25f * 3.14159265f; // 0..pi/2 の等パワーパン
  float l = cosf(angle), r = sinf(angle);
  for(int i = 0; i < fpip->audio_n; i++) {
    int16_t* s = &fpip->audiop[(size_t)i * 2];
    s[0]       = (int16_t)std::clamp((int32_t)(s[0] * l), -32768, 32767);
    s[1]       = (int16_t)std::clamp((int32_t)(s[1] * r), -32768, 32767);
  }
  return true;
}

bool pan_init(void* fp, ABIContext* editp, cutil::PropInfo* props, cutil::Prop* defaults) {
  MU_UNUSED(fp);
  MU_UNUSED(editp);
  props->fields.push_back(cutil::PropInfo::Field("pan", 0, cutil::prop_info_of<float>()));
  props->fields.back().set_label("左右パン");
  props->fields.back().min_value  = -100.0f;
  props->fields.back().max_value  = 100.0f;
  props->fields.back().drag_speed = 1.0f;
  defaults->set<float>("pan", 0.0f);
  return true;
}

} // namespace

FilterPluginTable f_audio_volume = {
  GUID(0x0001),                           // id
  FilterAudioOnly,                        // flag
  cutil::Str("音量調整"),                 // name
  cutil::Str("音声トラックの音量を調整"), // desc
  0,
  "0",
  nullptr,
  nullptr,
  volume_init,
  nullptr,
  volume_proc,
  nullptr,
  nullptr,
  nullptr,
  nullptr,
};

FilterPluginTable f_audio_pan = {
  GUID(0x0002), FilterAudioOnly, cutil::Str("左右パン"), cutil::Str("音声トラックの左右バランスを調整"), 0, "0", nullptr, nullptr, pan_init, nullptr, pan_proc, nullptr, nullptr, nullptr, nullptr,
};

} // namespace mu::detail
