#include <cstdio>
#include <doctest/doctest.h>
#include <movutl/audio/waveform_cache.hpp>
#include <movutl/core/filesystem.hpp>

using namespace mu;

TEST_CASE("fs_cache_key: 同一入力なら同一出力") {
  auto a = fs_cache_key("foo.wav", 12345);
  auto b = fs_cache_key("foo.wav", 12345);
  CHECK(a == b);
}

TEST_CASE("fs_cache_key: pathかmtimeが違えば出力も変わる") {
  auto base = fs_cache_key("foo.wav", 12345);
  CHECK(base != fs_cache_key("bar.wav", 12345));
  CHECK(base != fs_cache_key("foo.wav", 99999));
}

TEST_CASE("save_waveform_cache/load_waveform_cache: ラウンドトリップ") {
  WaveformData data;
  data.fps    = 30.0f;
  data.levels = {0, 10, 100, 200, 255};

  const std::string path = "cache/waveform_cache_test_roundtrip.bin";
  if(!fs_exists("cache")) fs_create_directory("cache");
  REQUIRE(save_waveform_cache(path, data));

  WaveformData loaded;
  REQUIRE(load_waveform_cache(path, &loaded));
  CHECK(loaded.fps == doctest::Approx(30.0f));
  CHECK(loaded.levels == data.levels);

  std::remove(path.c_str());
}

TEST_CASE("load_waveform_cache: マジック不一致のファイルはfalseを返す") {
  const std::string path = "cache/waveform_cache_test_broken.bin";
  if(!fs_exists("cache")) fs_create_directory("cache");
  FILE* f = std::fopen(path.c_str(), "wb");
  REQUIRE(f != nullptr);
  const char junk[] = "not a waveform cache file";
  std::fwrite(junk, 1, sizeof(junk), f);
  std::fclose(f);

  WaveformData loaded;
  CHECK_FALSE(load_waveform_cache(path, &loaded));

  std::remove(path.c_str());
}

TEST_CASE("load_waveform_cache: 存在しないファイルはfalseを返す") {
  WaveformData loaded;
  CHECK_FALSE(load_waveform_cache("cache/does_not_exist_waveform.bin", &loaded));
}
