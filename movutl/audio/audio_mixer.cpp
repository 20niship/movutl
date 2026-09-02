#include <algorithm>
#include <chrono>
#include <cstring>
#include <maximilian.h>
#include <movutl/asset/audio.hpp>
#include <movutl/asset/composition.hpp>
#include <movutl/audio/audio_mixer.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/core/profiler.hpp>
#include <movutl/plugin/filter.hpp>

namespace mu {

AudioRingBuffer::AudioRingBuffer(int32_t sample_rate, int32_t channels, double capacity_seconds) : sample_rate_(sample_rate), channels_(channels), capacity_samples_((int64_t)(sample_rate * capacity_seconds)) { ring_.assign((size_t)capacity_samples_ * std::max(1, channels_), 0); }

void AudioRingBuffer::write(int64_t from_sample, const int16_t* data, int n) {
  std::lock_guard<std::mutex> lock(mtx_);
  for(int i = 0; i < n; i++) {
    int64_t s   = from_sample + i;
    int64_t pos = ((s % capacity_samples_) + capacity_samples_) % capacity_samples_;
    std::memcpy(&ring_[(size_t)pos * channels_], &data[(size_t)i * channels_], sizeof(int16_t) * channels_);
  }
  int64_t end = from_sample + n;
  if(end > write_head_.load()) write_head_.store(end);
}

void AudioRingBuffer::snapshot(int64_t from_sample, int n, int16_t* out) const {
  std::lock_guard<std::mutex> lock(mtx_);
  int64_t head = write_head_.load();
  for(int i = 0; i < n; i++) {
    int64_t s = from_sample + i;
    if(s < 0 || s >= head || s < head - capacity_samples_) {
      std::memset(&out[(size_t)i * channels_], 0, sizeof(int16_t) * channels_);
      continue;
    }
    int64_t pos = ((s % capacity_samples_) + capacity_samples_) % capacity_samples_;
    std::memcpy(&out[(size_t)i * channels_], &ring_[(size_t)pos * channels_], sizeof(int16_t) * channels_);
  }
}

void AudioRingBuffer::seek(int64_t sample) {
  std::lock_guard<std::mutex> lock(mtx_);
  write_head_.store(sample);
  read_cursor_.store(sample);
}

int AudioRingBuffer::read_consume(int16_t* out, int n) {
  int64_t cur  = read_cursor_.load();
  int64_t head = write_head_.load();
  if(cur + n > head) {
    // アンダーラン検出。リアルタイムスレッドでの頻発を避けるため200ms間隔に抑制してログする
    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    int64_t last   = last_underrun_log_ms_.load();
    if(now_ms - last > 200 && last_underrun_log_ms_.compare_exchange_strong(last, now_ms)) LOG_F(WARNING, "AudioRingBuffer: underrun, short by %lld samples (read_cursor=%lld, write_head=%lld)", (long long)(cur + n - head), (long long)cur, (long long)head);
  }
  snapshot(cur, n, out);
  read_cursor_.store(cur + n);
  return n;
}

AudioMixWorker::AudioMixWorker() {
  thread_ = std::thread([this] { worker_loop(); });
}

AudioMixWorker::~AudioMixWorker() { stop(); }

void AudioMixWorker::stop() {
  if(stop_.exchange(true)) return;
  if(thread_.joinable()) thread_.join();
}

void AudioMixWorker::tick(Composition* comp, bool playing) {
  comp_.store(comp);
  playing_.store(playing);
}

void mix_audio_range(Composition* comp, int64_t start_sample, int n, int16_t* out) {
  MOVUTL_ZONE_SCOPED_N("mix_audio_range");
  maxiSettings::setup(comp->audio_sample_rate, comp->audio_channels, n);
  int ch = std::max(1, comp->audio_channels);
  std::fill(out, out + (size_t)n * ch, (int16_t)0);
  std::vector<int16_t> track_buf((size_t)n * ch, 0);

  int frame = (int)((double)start_sample / std::max(1, comp->audio_sample_rate) * comp->framerate); // このチャンクの時刻に対応するフレーム(現在再生フレームではない)

  auto entities = comp->get_all_entities();
  bool any_solo = false;
  for(auto& e : entities) {
    if(e->getType() == EntityType_Audio && e->visible(frame) && e->trk.solo_) {
      any_solo = true;
      break;
    }
  }

  for(auto& e : entities) {
    if(e->getType() != EntityType_Audio) continue;
    auto* a = static_cast<AudioEntt*>(e.get());
    if(!a->visible(frame)) continue;
    if(any_solo && !a->trk.solo_) continue; // ソロ中のトラックが1つでもあれば、ソロでないトラックはミュートする

    std::fill(track_buf.begin(), track_buf.end(), (int16_t)0);
    if(!a->fetch_audio(comp, start_sample, n, track_buf.data())) continue;

    for(auto& f : a->trk.filters) {
      if(!f.enabled || f.plg_ == nullptr || f.plg_->fn_proc == nullptr) continue;
      MOVUTL_ZONE_SCOPED;
      MOVUTL_ZONE_NAME(f.plg_->name.c_str(), f.plg_->name.size());
      FilterInData fin;
      fin.audiop   = track_buf.data();
      fin.audio_n  = n;
      fin.audio_ch = ch;
      fin.compo    = comp;
      fin.entt     = a;
      f.plg_->fn_proc(&f.instance_state, &fin, f.props.get(frame));
    }

    for(int i = 0; i < n * ch; i++) out[i] = (int16_t)std::clamp((int32_t)out[i] + (int32_t)track_buf[i], -32768, 32767);
  }
}

void AudioMixWorker::mix_range(Composition* comp, int64_t start_sample, int n) {
  int ch = std::max(1, comp->audio_channels);
  std::vector<int16_t> master((size_t)n * ch, 0);
  mix_audio_range(comp, start_sample, n, master.data());
  comp->audio_buf->write(start_sample, master.data(), n);
}

void AudioMixWorker::worker_loop() {
  constexpr int kChunkMs = 20;
  while(!stop_.load()) {
    Composition* comp = comp_.load();
    if(!comp || !comp->audio_buf) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kChunkMs));
      continue;
    }

    // 基準はComposition::frame(GUI描画で律速され重いシーンでは実時間より遅れうる)ではなくread_cursor(実際の再生位置)。frame基準だと先読み済みと誤判定して音切れする
    int64_t play_pos  = comp->audio_buf->read_cursor();
    int64_t head      = comp->audio_buf->write_head();
    int64_t lookahead = comp->audio_sample_rate; // 常時1秒分先読みしておく
    if(head < play_pos) head = play_pos;         // シーク直後などバッファが再生位置より遅れている場合は追いつく

    if(head - play_pos >= lookahead) {
      // 先読み十分な時だけ待機する(先頭で無条件sleepだと生産速度が常に等速止まりになり、一度遅れると二度と追いつけない)
      std::this_thread::sleep_for(std::chrono::milliseconds(kChunkMs));
      continue;
    }

    int chunk_samples = comp->audio_sample_rate * kChunkMs / 1000;
    mix_range(comp, head, chunk_samples);
  }
}

} // namespace mu
