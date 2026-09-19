#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>
//
#include <algorithm>
#include <atomic>
#include <cstring>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/audio/audio_player.hpp>
#include <movutl/core/logger.hpp>

namespace mu {

namespace {

// オーディオスレッドから読むためファイルスコープのatomicに持つ(デバイスは1つだけ)
std::atomic<float> g_master_gain{1.0f};

void data_callback(ma_device* device, void* output, const void* input, ma_uint32 frame_count) {
  (void)input;
  auto* comp = (Composition*)device->pUserData;
  if(!comp || !comp->audio_buf) {
    std::memset(output, 0, (size_t)frame_count * device->playback.channels * sizeof(int16_t));
    return;
  }
  int n = comp->audio_buf->read_consume((int16_t*)output, (int)frame_count);
  (void)n;
  const float g = g_master_gain.load(std::memory_order_relaxed);
  if(g != 1.0f) {
    auto* out          = (int16_t*)output;
    const size_t count = (size_t)frame_count * device->playback.channels;
    for(size_t i = 0; i < count; i++) out[i] = (int16_t)std::clamp((int)(out[i] * g), -32768, 32767);
  }
}

} // namespace

void AudioPlayer::set_master_gain(float gain) { g_master_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
float AudioPlayer::master_gain() { return g_master_gain.load(std::memory_order_relaxed); }

AudioPlayer::AudioPlayer() {}

AudioPlayer::~AudioPlayer() {
  stop();
  if(device_) delete(ma_device*)device_;
}

void AudioPlayer::set_composition(Composition* comp) {
  bool was_running = running_;
  stop();
  comp_ = comp;
  if(device_) {
    delete(ma_device*)device_;
    device_ = nullptr;
  }
  if(!comp_) return;

  auto* device          = new ma_device();
  ma_device_config cfg  = ma_device_config_init(ma_device_type_playback);
  cfg.playback.format   = ma_format_s16;
  cfg.playback.channels = (ma_uint32)comp_->audio_channels;
  cfg.sampleRate        = (ma_uint32)comp_->audio_sample_rate;
  cfg.dataCallback      = data_callback;
  cfg.pUserData         = comp_;

  if(ma_device_init(nullptr, &cfg, device) != MA_SUCCESS) {
    LOG_F(ERROR, "AudioPlayer: failed to init playback device");
    delete device;
    device_ = nullptr;
    return;
  }
  device_ = device;
  if(was_running) start();
}

void AudioPlayer::start() {
  if(!device_ || running_) return;
  if(ma_device_start((ma_device*)device_) == MA_SUCCESS) running_ = true;
}

void AudioPlayer::stop() {
  if(!device_ || !running_) return;
  ma_device_stop((ma_device*)device_);
  running_ = false;
}

} // namespace mu
