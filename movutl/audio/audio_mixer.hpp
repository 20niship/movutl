#pragma once

#include <atomic>
#include <cstdint>
#include <cutil/ref.hpp>
#include <mutex>
#include <thread>
#include <vector>

namespace mu {

class Composition;

// Compositionの[start_sample, start_sample+n)分の音声をミックスしoutへ書き込む(PCM16 interleaved)。AudioMixWorkerとexport処理が共用する
void mix_audio_range(Composition* comp, int64_t start_sample, int n, int16_t* out);

// 固定長circular buffer。PCM16bit interleaved(FilterInData::audiopと同一形式)
class AudioRingBuffer {
public:
  AudioRingBuffer(int32_t sample_rate, int32_t channels, double capacity_seconds = 2.0);

  int32_t sample_rate() const { return sample_rate_; }
  int32_t channels() const { return channels_; }
  int64_t capacity_samples() const { return capacity_samples_; }
  int64_t write_head() const { return write_head_.load(); }

  // 絶対サンプル位置(from_sample)からnサンプル分書き込む(ミキシングスレッド専用)
  void write(int64_t from_sample, const int16_t* data, int n);

  // 絶対サンプル位置からnサンプル分読み取る。範囲外/未書き込み分は無音(0)埋め。読み取りカーソルは進めない
  void snapshot(int64_t from_sample, int n, int16_t* out) const;

  // 再生カーソルからnサンプル読み取り、カーソルを進める(再生デバイスコールバック専用)。戻り値は実際に読めたサンプル数
  int read_consume(int16_t* out, int n);

  int64_t read_cursor() const { return read_cursor_.load(); }

  // シーク時に読み書きカーソル双方を指定サンプル位置へ飛ばす(ミキサーはここから再度前方へ書き直す)
  void seek(int64_t sample);

private:
  int32_t sample_rate_;
  int32_t channels_;
  int64_t capacity_samples_;
  std::vector<int16_t> ring_;          // capacity_samples_ * channels_
  std::atomic<int64_t> write_head_{0}; // 直近書き込んだ絶対サンプル位置の次(バッファに反映済みの範囲の終端)
  std::atomic<int64_t> read_cursor_{0};
  mutable std::mutex mtx_;
};

// 複数の音声トラックを別スレッドで合成し、Composition::audio_bufへ書き込むワーカー
class AudioMixWorker {
public:
  AudioMixWorker();
  ~AudioMixWorker();

  AudioMixWorker(const AudioMixWorker&)            = delete;
  AudioMixWorker& operator=(const AudioMixWorker&) = delete;

  // RenderWorkerPool::tickと同じ呼び出し規約: 毎フレームGUIループから呼ぶ
  void tick(Composition* comp, bool playing);
  void stop();

private:
  void worker_loop();
  // 現在のcomp_について指定サンプル範囲のミックス結果をaudio_bufへ書き込む
  void mix_range(Composition* comp, int64_t start_sample, int n);

  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> playing_{false};
  std::atomic<Composition*> comp_{nullptr};
};

} // namespace mu
