#include <algorithm>
#include <cmath>
#include <movutl/asset/compo_audio_ref.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/audio_resample.hpp>
#include <vector>

namespace mu {

bool CompoAudioEntt::render(Composition* cmp, Image* target, int frame) {
  MU_UNUSED(cmp);
  MU_UNUSED(target);
  MU_UNUSED(frame);
  return true; // 音声Enttは画像を描画しない
}

bool CompoAudioEntt::fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out) {
  MU_ASSERT(cmp != nullptr);
  MU_ASSERT(out != nullptr);
  if(!trk.active_) return false;

  Composition* dst_comp = nullptr;
  for(auto& c : Project::Get()->compos_) {
    if(c->guid == target_comp_guid) {
      dst_comp = c.get();
      break;
    }
  }
  if(!dst_comp || dst_comp == cmp) return false;
  if(!Composition::PushRenderGuard(dst_comp->guid)) return false;

  // RAIIでPopRenderGuardの呼び忘れを防ぐ(以降のreturnは全てこのguardのデストラクタを通る)
  struct Guard {
    Composition* c;
    ~Guard() { Composition::PopRenderGuard(c->guid); }
  } guard{dst_comp};

  int64_t track_start = cmp->frame_to_sample(trk.fstart);
  int64_t track_len   = cmp->frame_to_sample(trk.fend) - track_start;
  if(track_len <= 0) return false;
  int64_t elapsed = start_sample - track_start;
  if(elapsed + n <= 0 || elapsed >= track_len) return false;

  double speed_ratio       = std::max(0.01, (double)speed);
  int64_t dst_start_sample = (int64_t)((double)elapsed / cmp->audio_sample_rate * dst_comp->audio_sample_rate * speed_ratio) + dst_comp->frame_to_sample(start_frame);
  int dst_n                = std::max(1, (int)std::ceil((double)n * dst_comp->audio_sample_rate * speed_ratio / cmp->audio_sample_rate) + 2);

  std::vector<int16_t> dst_buf((size_t)dst_n * std::max(1, dst_comp->audio_channels), 0);
  mix_audio_range(dst_comp, dst_start_sample, dst_n, dst_buf.data());

  std::vector<int16_t> resampled((size_t)n * cmp->audio_channels, 0);
  double rate_ratio = (double)cmp->audio_sample_rate / ((double)dst_comp->audio_sample_rate * speed_ratio);
  audio_resample(dst_buf.data(), dst_n, dst_comp->audio_channels, resampled.data(), n, cmp->audio_channels, rate_ratio);

  float gain = volume_ / 100.0f;
  for(int i = 0; i < n * cmp->audio_channels; i++) {
    int32_t mixed = out[i] + (int32_t)(resampled[i] * gain);
    out[i]        = (int16_t)std::clamp(mixed, -32768, 32767);
  }
  return true;
}

} // namespace mu
