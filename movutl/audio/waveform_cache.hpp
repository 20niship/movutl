#pragma once
#include <cstdint>
#include <movutl/asset/entity.hpp>
#include <string>
#include <vector>

namespace mu {

struct InputPluginTable;

// タイムライン表示用の音量波形データ。1エントリ = 1/fps秒ぶんのRMS音量(0-255)
struct WaveformData {
  float fps = 30.0f;
  std::vector<uint8_t> levels;

  int index_for_second(double sec) const { return (int)(sec * fps); }
};

// pathの音声ファイルに対応するキャッシュファイルパス(cache/{hash}.bin)を返す。cache/ディレクトリが無ければ作成する
std::string waveform_cache_path(const std::string& path);

// キャッシュファイルを読み込む。存在しない/壊れている場合はfalse
bool load_waveform_cache(const std::string& cache_path, WaveformData* out);

// キャッシュファイルへ保存する
bool save_waveform_cache(const std::string& cache_path, const WaveformData& data);

// plg->fn_read_audioで全サンプルを読み、1/fpsごとのRMS音量を計算してoutへ格納する
bool generate_waveform(InputPluginTable* plg, InputHandle handle, int32_t audio_n, int32_t sample_rate, int32_t channels, float fps, WaveformData* out);

} // namespace mu
