#include <algorithm>
#include <cmath>
#include <cstdio>
#include <movutl/audio/waveform_cache.hpp>
#include <movutl/core/filesystem.hpp>
#include <movutl/plugin/input.hpp>
#include <vector>

namespace mu {

namespace {
struct WaveformFileHeader {
  char magic[4]     = {'M', 'W', 'F', 'C'};
  uint32_t version  = 1;
  float fps         = 30.0f;
  uint32_t channels = 0;
  uint32_t count    = 0;
};
} // namespace

std::string waveform_cache_path(const std::string& path) {
  if(!fs_exists("cache")) fs_create_directory("cache");
  int64_t mtime = fs_last_write_time_raw(path);
  return "cache/" + fs_cache_key(path, mtime) + ".bin";
}

bool load_waveform_cache(const std::string& cache_path, WaveformData* out) {
  if(!out || !fs_is_file(cache_path)) return false;
  FILE* f = std::fopen(cache_path.c_str(), "rb");
  if(!f) return false;

  WaveformFileHeader header;
  bool ok = std::fread(&header, sizeof(header), 1, f) == 1;
  if(ok && (header.magic[0] != 'M' || header.magic[1] != 'W' || header.magic[2] != 'F' || header.magic[3] != 'C' || header.version != 1)) ok = false;

  if(ok) {
    out->fps = header.fps;
    out->levels.assign(header.count, 0);
    if(header.count > 0 && std::fread(out->levels.data(), 1, header.count, f) != header.count) ok = false;
  }
  std::fclose(f);
  return ok;
}

bool save_waveform_cache(const std::string& cache_path, const WaveformData& data) {
  FILE* f = std::fopen(cache_path.c_str(), "wb");
  if(!f) return false;

  WaveformFileHeader header;
  header.fps      = data.fps;
  header.channels = 0;
  header.count    = (uint32_t)data.levels.size();

  bool ok = std::fwrite(&header, sizeof(header), 1, f) == 1;
  if(ok && header.count > 0) ok = std::fwrite(data.levels.data(), 1, header.count, f) == header.count;
  std::fclose(f);
  return ok;
}

bool generate_waveform(InputPluginTable* plg, InputHandle handle, int32_t audio_n, int32_t sample_rate, int32_t channels, float fps, WaveformData* out) {
  if(!plg || !handle || !plg->fn_read_audio || audio_n <= 0 || sample_rate <= 0 || channels <= 0 || fps <= 0 || !out) return false;

  out->fps = fps;
  out->levels.clear();

  int samples_per_bucket = std::max(1, (int)(sample_rate / fps));
  std::vector<int16_t> buf((size_t)samples_per_bucket * channels);

  for(int64_t start = 0; start < audio_n; start += samples_per_bucket) {
    int n    = (int)std::min((int64_t)samples_per_bucket, audio_n - start);
    int read = plg->fn_read_audio(handle, (int)start, n, buf.data());
    if(read <= 0) {
      out->levels.push_back(0);
      continue;
    }

    double sum_sq = 0.0;
    int64_t count = (int64_t)read * channels;
    for(int64_t i = 0; i < count; i++) sum_sq += (double)buf[i] * buf[i];
    double rms = count > 0 ? std::sqrt(sum_sq / count) : 0.0;
    out->levels.push_back((uint8_t)std::clamp(rms / 32768.0 * 255.0, 0.0, 255.0));
  }
  return true;
}

} // namespace mu
