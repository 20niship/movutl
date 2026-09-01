#include <algorithm>
#include <cmath>
#include <movutl/app/app.hpp>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/asset/project.hpp>
#include <movutl/core/audio_resample.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <vector>

namespace mu {

AudioEntt::AudioEntt(const char* path) {
  if(path != nullptr && path[0] != '\0') load_file(path);
}

Ref<AudioEntt> AudioEntt::Create(const char* name, const char* path) {
  auto a  = cutil::make_ref<AudioEntt>();
  a->name = name;
  Project::Get()->entities.push_back(a);
  a->guid_ = Project::Get()->entities.size();
  if(path != nullptr && path[0] != '\0') a->load_file(path);
  return a;
}

bool AudioEntt::load_file(const char* path) {
  if(in_plg_ != nullptr && in_handle_ != nullptr) in_plg_->fn_close(in_handle_);
  in_plg_      = nullptr;
  in_handle_   = nullptr;
  info         = EntityInfo();
  load_failed_ = true;

  auto p = get_compatible_plugin(path, EntityType_Audio);
  if(p == nullptr) {
    LOG_F(ERROR, "No compatible audio plugin found for file: %s", path);
    return false;
  }
  this->path_ = path;

  in_handle_ = p->fn_open(path);
  if(in_handle_ == nullptr) {
    LOG_F(ERROR, "Failed to open audio file: %s", path);
    return false;
  }
  in_plg_ = p;

  if(p->fn_info_get == nullptr || !p->fn_info_get(in_handle_, &info) || info.audio_n <= 0) {
    LOG_F(ERROR, "Failed to get audio info: %s -> %s", path, info.str().c_str());
    return false;
  }

  auto* cmp           = Composition::GetActiveComp();
  float fps           = cmp ? cmp->framerate : 30.0f;
  double duration_sec = (double)info.audio_n / std::max(1, info.audio_sample_rate);
  this->trk.fend      = trk.fstart + (int)std::max(1.0, duration_sec * fps);
  load_failed_        = false;
  LOG_F(INFO, "Audio loaded: %s (%d samples, %d Hz, %d ch, plugin=%s)", path, info.audio_n, info.audio_sample_rate, info.audio_channels, p->name);
  return true;
}

bool AudioEntt::render(Composition* cmp, Image* target, int frame) {
  MU_UNUSED(cmp);
  MU_UNUSED(target);
  MU_UNUSED(frame);
  return true; // 音声Enttは画像を描画しない
}

bool AudioEntt::fetch_audio(Composition* cmp, int64_t start_sample, int n, int16_t* out) {
  MU_ASSERT(cmp != nullptr);
  MU_ASSERT(out != nullptr);
  if(load_failed_ || !trk.active_ || in_plg_ == nullptr || in_handle_ == nullptr || in_plg_->fn_read_audio == nullptr) return false;
  if(info.audio_n <= 0) return false;

  int64_t track_start = cmp->frame_to_sample(trk.fstart);
  int64_t track_len   = cmp->frame_to_sample(trk.fend) - track_start;
  if(track_len <= 0) return false;
  int64_t elapsed = start_sample - track_start;
  if(elapsed + n <= 0) return false;
  if(elapsed >= track_len) {
    if(!loop_) return false;
    elapsed %= track_len;
  }

  int32_t native_rate = info.audio_sample_rate > 0 ? info.audio_sample_rate : cmp->audio_sample_rate;
  int32_t native_ch   = info.audio_channels > 0 ? info.audio_channels : cmp->audio_channels;
  double speed_ratio  = std::max(0.01, (double)speed / 100.0);

  int64_t native_start = (int64_t)((double)elapsed / cmp->audio_sample_rate * native_rate * speed_ratio) + (int64_t)(offset_sec_ * native_rate);
  int native_n         = std::max(1, (int)std::ceil((double)n * native_rate * speed_ratio / cmp->audio_sample_rate) + 2);

  std::vector<int16_t> native_buf((size_t)native_n * native_ch, 0);
  int read = in_plg_->fn_read_audio(in_handle_, (int)native_start, native_n, native_buf.data());
  if(read <= 0) return false;

  std::vector<int16_t> resampled((size_t)n * cmp->audio_channels, 0);
  double rate_ratio = (double)cmp->audio_sample_rate / ((double)native_rate * speed_ratio);
  audio_resample(native_buf.data(), read, native_ch, resampled.data(), n, cmp->audio_channels, rate_ratio);

  float gain = volume_ / 100.0f;
  for(int i = 0; i < n * cmp->audio_channels; i++) {
    int32_t mixed = out[i] + (int32_t)(resampled[i] * gain);
    out[i]        = (int16_t)std::clamp(mixed, -32768, 32767);
  }
  return true;
}

} // namespace mu
